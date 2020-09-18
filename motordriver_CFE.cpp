// System includes
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <chrono>
#include <thread>
#include <boost/algorithm/string/replace.hpp>

#include "motordriver.hpp"

extern "C" {
    #include <i2c/smbus.h>
    #include <linux/i2c-dev.h>
    #include <linux/i2c.h>
}

using namespace std;

static void _mlog(std::string in) { std::cout << in << std::endl;  };

namespace ImageServer
{

// Addresses of the chips on the I2C bus
int MotorDriver_CFE::addr1 = 0x20;  //U4 (Zoom, focus)
int MotorDriver_CFE::addr2 = 0x21;  //U5 (Power on mainboard)
int MotorDriver_CFE::addr0 = 0x22;  //U7 (Iris/Ircut)

// This value will change if the lens changes, based on the speed of the lens motor.
#define STEPPER_SLEEP_TIME_MICRO 1667 // 600 pps

#define ZOOM_LIMIT      0x01 //pin U5.A.0
#define ZOOM_DIR        0x02 //pin U5.A.1
#define ZOOM_ENABLE     0x04 //pin U5.A.2
#define ZOOM_STEP       0x08 //pin U5.A.3
#define ZOOM_FAULT      0x10 //pin U5.A.4
#define ZOOM_MS2        0X20 //pin U5.A.5
#define ZOOM_MS1	    0x40 //pin U5.A.6
#define ZOOM_SLEEP	    0x80 //pin U5.A.7
#define ZOOM_AENBL 	    ZOOM_ENABLE
#define ZOOM_BENBL 	    ZOOM_STEP
#define ZOOM_APHASE 	ZOOM_MS1
#define ZOOM_BPHASE	    ZOOM_DIR
#define ZOOM_EN1	    ZOOM_MS2

#define FOCUS_LIMIT     0x01 //pin U5.B.0
#define FOCUS_DIR       0x02 //pin U5.B.1
#define FOCUS_ENABLE    0x04 //pin U5.B.2
#define FOCUS_STEP      0x08 //pin U5.B.3
#define FOCUS_FAULT     0x10 //pin U5.B.4
#define FOCUS_MS2       0X20 //pin U5.B.5
#define FOCUS_MS1       0x40 //pin U5.B.6
#define FOCUS_SLEEP     0x80 //pin U5.B.7
#define FOCUS_AENBL     FOCUS_ENABLE
#define FOCUS_BENBL     FOCUS_STEP
#define FOCUS_APHASE    FOCUS_MS1
#define FOCUS_BPHASE    FOCUS_DIR
#define FOCUS_EN1       FOCUS_MS2

#define PG		        0x01 //pin U6.A.0
#define IRIS_DIR        0x02 //pin U6.A.1
#define IRIS_ENABLE     0x04 //pin U6.A.2
#define IRIS_STEP       0x08 //pin U6.A.3
#define IRIS_FAULT      0x10 //pin U6.A.4
#define IRIS_MS2        0X20 //pin U6.A.5
#define IRIS_MS1        0x40 //pin U6.A.6
#define IRIS_SLEEP      0x80 //pin U6.A.7
#define IRIS_AENBL      IRIS_ENABLE
#define IRIS_BENBL      IRIS_STEP
#define IRIS_APHASE     IRIS_MS1
#define IRIS_BPHASE     IRIS_DIR
#define IRIS_EN1        IRIS_MS2


#define POWER_5V_EN_LDO     0x02 //pin U5.A.1
#define PGOOD		        0x01 //pin U5.A.0
#define POWER_4V_EN         0x80 //pin U5.B.7
#define POWER_2_8V_EN       0x40 //pion U5.B.6
#define POWER_1_8V_EN       0x20 //pin U5.B.5
#define POWER_1_2V_EN       0x10 //pin U5.B.4
#define POWER_5V_EN         0x08 //pin U5.B.3
#define PG_4V		        0x04 //pin U5.B.2
#define POWER_3_3V_EN       0x02 //pin U5.B.1
#define PG_3_3V	            0x01 //pin U5.B.0


#define IRCUT_A		        0x02 //pin U6.B.1
#define IRCUT_B		        0x01 //pin U6.B.0

#define REG_IOCON   0x0A
#define REG_IODIRA  0x00
#define REG_IODIRB  0x01
#define REG_GPPUA   0x0C
#define REG_GPPUB   0x0D
#define REG_GPIOA   0x12
#define REG_GPIOB   0x13

#define REG_IOCON_VAL   0x00 // Output direction is value 0
#define REG_IODIRA_EXP1 0x11 //Fault, Pgood
#define REG_IODIRB_EXP1 0x11 //Fault, Pgood
#define REG_IODIRA_EXP0 0x00  
#define REG_IODIRB_EXP0 0xE0 //Board ID
#define REG_IODIRA_EXP2 0x01 //PGood, all others output
#define REG_IODIRB_EXP2 (0x01 + 0x04) // U5.B.0,3 are input, all others output
#define REG_GPPUA_VAL   0x00 // disable all pull-ups
#define REG_GPPUB_VAL   0x00
#define REG_GPIOA_EXP1  (ZOOM_SLEEP ) // wake up on init
#define REG_GPIOB_EXP1  (FOCUS_SLEEP)
#define REG_GPIOA_EXP2  0x00
#define REG_GPIOB_EXP2  (POWER_1_8V_EN) //keep 1.8V on so level translator will work.
#define REG_GPIOA_EXP0  (IRIS_SLEEP ) // wake up on init
#define REG_GPIOB_EXP0  0x00


// Values as per the Theia-TL410P datasheet. 
const  struct MotorDriverBase::lens_settings MotorDriver_CFE::LensSettingsDefault_CFE_theia_tl410p =
{
    .focus =  { .min = 0, .max= 9354 },
    .zoom = { .min=0, .max= 4073 },
    .iris = { .min=0, .max= 75 }
};


#define DO_WRITE(addr,reg,data) \
	{if (false == writeReg(addr,reg,data)) { ostringstream oss; oss << "Write failed for addr: " << hex << addr << " reg: " << hex << reg << dec; _mlog(oss.str()); return false;}}

    MotorDriver_CFE::MotorDriver_CFE(bool doInit, boost::function < void(std::string) > log_callback):
        MotorDriverBase(MotorDriver_CFE::LensSettingsDefault_CFE_theia_tl410p),
        fd_(-1)
{
    if (true == doInit) {
        init();
        //enablePowerLines();
        // Home our zoom and then move to start position
        //if (false == (zoom_init = zoomHome())) {
        //    mlog("Unable to home zoom");
        //} else if (false == zoomAbsolute(Configuration::zoom_start())) {
        //    ostringstream oss;
        //    oss << "Unable to zoom to start position: " << Configuration::zoom_start();
        //    mlog(oss.str());
        //}

        // Home our focus and then move to start position
        /*if (false == (focus_init = focusHome())) {
            mlog("Unable to home focus");
        } else if (false == focusAbsolute(Configuration::focus_start())) {
            ostringstream oss;
            oss << "Unable to focus to start position: " << Configuration::focus_start();
            mlog(oss.str());
        }*/
        //irisHome();
    }
}

MotorDriver_CFE::~MotorDriver_CFE()
{}

// Initialization function for I2C with sensor module.
bool MotorDriver_CFE::initImplementation()
{
    _mlog("Initializing i2c-6 device");
    std::string devicename("/dev/i2c-6");
    if ((fd_ = open(devicename.c_str(), O_RDWR)) < 0) {
        ostringstream oss;
        oss << "Unable to open device: " << devicename;
        _mlog(oss.str());
        return false;
    }
    uint64_t funcs;
    if (ioctl(fd_, I2C_FUNCS, &funcs) < 0) {
        _mlog("Unable to get I2C_FUNCS");
	return false;
    }
    if (funcs & I2C_FUNC_I2C) {
        _mlog("Supports I2C_RDWR");
    } else if (funcs & I2C_FUNC_SMBUS_WORD_DATA) {
        _mlog("Does not support I2C_RDWR");
    } else {
        _mlog("Unable to get valid FUNC");
        return false;
    }

    return initExpanders();
}


/*
* @brief: Write function for I2C communicating with sensor module.
* @param1: Slave address
* @param2: Register address to write to.
* @param3: Data to write to the register address.
*/
bool MotorDriver_CFE::writeReg(uint8_t addr, uint8_t regaddr, uint8_t data)
{
    ostringstream oss;
    oss << ": 0x" << hex << std::setw(2) << std::setfill('0') << (unsigned)addr << " reg 0x" << std::setw(2) << std::setfill('0') << (unsigned)regaddr << " val 0x" << std::setw(2) << std::setfill('0') << (unsigned)data << dec;
    _mlog(oss.str());
    if (ioctl(fd_, I2C_SLAVE, addr) < 0) {
        ostringstream oss;
        oss << "Unable to access slave: " << addr;
        _mlog(oss.str());
        return false;
    }
    char buf[10];
    buf[0] = regaddr;
    buf[1] = data;
    if (write(fd_, buf, 2) != 2) {
        _mlog("Unable to write data");
        return false;
    }
    return true;
}


/* 
* @bried: Read function for I2C communicating with sensor module.
* @param1: Slave address
* @param2: Register address to read from.
* @param3: Pointer to the variable to store read data.
*/
bool MotorDriver_CFE::readReg(uint8_t addr, uint8_t regaddr, uint8_t& res)
{
    if (ioctl(fd_, I2C_SLAVE, addr) < 0) {
        ostringstream oss;
        oss << "Unable to access slave: " << addr;
        _mlog(oss.str());
        return false;
    }
    res = i2c_smbus_read_byte_data(fd_, regaddr);
    return true;
}


/* 
* @brief: Prints the data read from a register address
* @param1: Slave address
* @param2: Register address to read from.
*/
bool MotorDriver_CFE::printReg(uint8_t addr, uint8_t regaddr)
{
    uint8_t res = 0x00;
    bool ret = readReg(addr, regaddr, res);
    ostringstream oss;
    oss << "0x" << std::hex << std::setw(2) << std::setfill('0') << (unsigned)res << std::dec;
    _mlog(oss.str());
    return ret;
}

// Function to print Zoom Register
bool MotorDriver_CFE::printZoom()
{
    return printReg(addr1, REG_GPIOA);
}

// Function to print Focus Register
bool MotorDriver_CFE::printFocus()
{
    return printReg(addr1, REG_GPIOB);
}

// Function to detect the presence of gpio expander on board and use it if it does exist.
bool MotorDriver_CFE::initExpanders()
{
    _mlog("IN initExpanders");
    char buf[10] = {0};
    if (-1 == fd_) {
        _mlog("i2c device not initialized");
        return false;
    }

    if (pathExists("/sys/class/gpio/gpiochip240")) {
        use_gpio = true;
        gpioExport(247);
        gpioDirection(247, 1);
        gpioSet(247, 1);
    }
    else {
        _mlog("gpio expander not detected. Using i2c.");
        DO_WRITE(addr0, REG_IODIRA, REG_IODIRA_EXP0);
        DO_WRITE(addr0, REG_IODIRB, REG_IODIRB_EXP0);
        DO_WRITE(addr0, REG_GPPUA, REG_GPPUA_VAL);
        DO_WRITE(addr0, REG_GPPUB, REG_GPPUB_VAL);
        DO_WRITE(addr0, REG_GPIOA, REG_GPIOA_EXP0);
        exp0_gpioa = REG_GPIOA_EXP0;
        DO_WRITE(addr0, REG_GPIOB, REG_GPIOB_EXP0);
        exp0_gpiob = REG_GPIOB_EXP0;
    }

    DO_WRITE(addr1, REG_IODIRA, REG_IODIRA_EXP1);
    DO_WRITE(addr1, REG_IODIRB, REG_IODIRB_EXP1);
    DO_WRITE(addr1, REG_GPPUA, REG_GPPUA_VAL);
    DO_WRITE(addr1, REG_GPPUB, REG_GPPUB_VAL);

    DO_WRITE(addr1, REG_GPIOA, REG_GPIOA_EXP1);
    exp1_gpioa = REG_GPIOA_EXP1;

    DO_WRITE(addr1, REG_GPIOB, REG_GPIOB_EXP1);
    exp1_gpiob = REG_GPIOB_EXP1;

    if (false == enablePowerLines()) {
        _mlog("Unable to enable power lines");
        return false;
    }
    return true;
}

/*
    TODO: modify the enablePowerLines to work with 4.6 CFE
    Note: The Power Lines control is not possible with this code on <v4.6 hardware
*/
bool MotorDriver_CFE::enablePowerLines()
{
    // enable all power lines
    //exp2_gpiob |= (POWER_5V_EN | POWER_4V_EN | POWER_2_8V_EN | POWER_1_8V_EN | POWER_1_2V_EN);
    //DO_WRITE(addr2, REG_GPIOB, exp2_gpiob);
    return true;
}


/**********************************************************/
//Functions relevant to Zoom
/**********************************************************/

// Function to disable Zoom
bool MotorDriver_CFE::disableZoom()
{
    exp1_gpioa &= ~(ZOOM_AENBL + ZOOM_BENBL);
    DO_WRITE(addr1, REG_GPIOA, exp1_gpioa);
    return true;
}

// Function to enable Zoom
bool MotorDriver_CFE::enableZoom() {
    exp1_gpioa |= ZOOM_AENBL + ZOOM_BENBL;
    DO_WRITE(addr1, REG_GPIOA, exp1_gpioa);
}

// Function for zoomDown functionality
bool MotorDriver_CFE::zoomDown(int steps)
{
    bool modified = false;
    if(zoom_abs_location <= lens_motor_steps.zoom.min) {
	    zoom_abs_location = lens_motor_steps.zoom.min;
        modified = true;
        _mlog("Cannot zoomdown, reached minimum zoom setting");
    }
    else 
    {
	    enableZoom();
        if((int)(zoom_abs_location - steps) < (int)lens_motor_steps.zoom.min) {
            steps = zoom_abs_location;;
        }
        zoom(steps);
       	zoom_abs_location -= steps;
    	disableZoom();
        modified = true;
    }
    return modified;
}

// Function for zoomDown functioanlity
bool MotorDriver_CFE::zoomUp(int steps)
{
    bool modified = false;
    if(zoom_abs_location >= lens_motor_steps.zoom.max){
	    zoom_abs_location = lens_motor_steps.zoom.max;
        _mlog("Cannot zoomup, reached maximum zoom setting");
        modified = true;
    }
    else {
	    enableZoom();
 	    if (zoom_abs_location + steps > lens_motor_steps.zoom.max) {
            steps = lens_motor_steps.zoom.max - zoom_abs_location;
	    }
    	zoom(0-steps);
    	zoom_abs_location += steps;
    	disableZoom();
        modified = true;
    }
	return modified;
}


bool MotorDriver_CFE::zoomHome()
{
    bool ret = false;
    int steps = lens_motor_steps.zoom.max; //Configuration::zoom_home_max_steps();
    //if(steps < LOWER_LIMIT || steps > ZOOM_MAX_STEPS){
    //    steps = ZOOM_MAX_STEPS;
    //    _mlog("Warning: zoom steps cfg oob, setting steps to max size");
    //}
    const int stepsize = 5;
    enableZoom();
    int counter = 0;
    ostringstream oss;
    oss <<  " zoom limit pre while " << zoomLimit() << " stepsize " << stepsize;
    _mlog(oss.str());
    while ((!zoomLimit()) && (counter < steps)) {
        oss.str("");
        oss << "zoom limit during: " << zoomLimit();
        _mlog(oss.str());
        zoomUp(stepsize);
        counter += stepsize;
    }
    oss.str("");
    oss << "zoom limit after: " << zoomLimit();
    _mlog(oss.str());
    if (abs(counter) <= steps) {
        oss.str("");
        oss << "Reached zoom homing step limit of " << steps;
        _mlog(oss.str());
        zoom_abs_location = 0;
        zoom_init = true;
    } else {
        _mlog("Found zoom limit");
        zoom_abs_location = 0;
        ret = true;
    }
    disableZoom();
    return ret;
}

bool MotorDriver_CFE::zoomLimit()
{
    uint8_t res = 0x00;
    bool ret = readReg(addr1, REG_GPIOA, res);
    ostringstream oss;
    oss << "gpioa: 0x" << hex << (int)res << dec;
    _mlog(oss.str());
    return !(res & ZOOM_LIMIT);
}

// Function to rotate motor for zoom functionality as per the steps specified in the argument.
bool MotorDriver_CFE::zoom(int steps)
{
    char mask;
    char phase;
    char out;
    for(int i = 0; i < abs(steps); ++i) {
        phase = 0;
        if (steps >0) {
            phase = 3- (i%4);
        }
        else {
            phase = i%4;
        }
        out = 0;
        switch(phase){
		case 0:
			out = ZOOM_APHASE;
            break;
		case 1:
            out = ZOOM_APHASE+ ZOOM_BPHASE;
            break;
		case 2:
            out = ZOOM_BPHASE;
            break;	
		case 3:
			out=0;
            break;
        }
        mask = exp1_gpioa & (~(ZOOM_APHASE + ZOOM_BPHASE));  //set before to the mask of phase a and b and the current output register
        exp1_gpioa = out + mask; 
        DO_WRITE(addr1, REG_GPIOA, exp1_gpioa);
        std::this_thread::sleep_for(std::chrono::microseconds(STEPPER_SLEEP_TIME_MICRO));   
    }

    zoomLimit();
}


/**********************************************************/
//Functions relevant to Focus
/**********************************************************/

// Function to enable focus
bool MotorDriver_CFE::enableFocus() {
    exp1_gpiob |= FOCUS_AENBL + FOCUS_BENBL;
    DO_WRITE(addr1, REG_GPIOB, exp1_gpiob);
}

// Function to disable focus
bool MotorDriver_CFE::disableFocus()
{
    exp1_gpiob &= ~(FOCUS_AENBL+FOCUS_BENBL) ;
    DO_WRITE(addr1, REG_GPIOB, exp1_gpiob);
    return true;
}

// Function for focusUp functionality
bool MotorDriver_CFE::focusUp(int steps)
{	
    bool modified = false;
    if(focus_abs_location >= lens_motor_steps.focus.max) {
        focus_abs_location = lens_motor_steps.focus.max;
        _mlog("Cannot focusup, reached maximum focus setting");
        modified = true;
    }
    else 
    {
	    enableFocus();
        if(focus_abs_location + steps > lens_motor_steps.focus.max)
        {
            steps = lens_motor_steps.focus.max - focus_abs_location;
        }
	    focus(steps);
    	focus_abs_location += steps;
    	disableFocus();
        modified = true;
    }
    return modified;
}

// Function for focusDown functionality
bool MotorDriver_CFE::focusDown(int steps)
{
    bool modified = false;
    if(focus_abs_location <= lens_motor_steps.focus.min) {
	    focus_abs_location = lens_motor_steps.focus.min;
        _mlog("Cannot focusdown, reached minimum focus setting");
        modified = true;
    }
    else 
    {
	    enableFocus();
    	if((int)(focus_abs_location - steps) < (int)lens_motor_steps.focus.min)
        {
	        steps = focus_abs_location;
    	}
        focus(0-steps);
        focus_abs_location -= steps;
        disableFocus();
        modified = true;
    }
    return modified;
}

bool MotorDriver_CFE::focusHome()
{
    bool ret = false;
    int steps = lens_motor_steps.focus.max;
    if(steps < lens_motor_steps.focus.min || steps > lens_motor_steps.focus.max){
        steps = lens_motor_steps.focus.max;
        _mlog("Warning: focus steps cfg oob, setting steps to max size");
    }
    int stepsize = 4;
    enableFocus();
    int counter = 0;
    while ((!focusLimit()) && (abs(counter) < steps)) {
        focusDown(stepsize);
	    counter += stepsize;
    }
    if (abs(counter) <= steps) {
        ostringstream oss;
        oss << "Reached focus homing step limit of " << steps;
        _mlog(oss.str());
        focus_abs_location = 0;
        focus_init = true;
    } else {
        _mlog("Found focus limit");
        focus_abs_location = 0;
        ret = true;
    }
    disableFocus();
    return ret;
}


bool MotorDriver_CFE::focusLimit()
{
    uint8_t res = 0x00;
    bool ret = readReg(addr1, REG_GPIOB, res);
    ostringstream oss;
    oss << "gpiob: 0x" << hex << (int)res << dec;
    _mlog(oss.str());
    return (res & FOCUS_LIMIT);
}


// Function to rotate motor for focus functionality as per the steps specified in the argument.
bool MotorDriver_CFE::focus(int steps)
{
   char mask;
   char out;
   char phase;
   for(int i = 0; i < abs(steps); ++i) {
        phase = 0;
        if (steps <0) {
                phase = 3- (i%4);
        }
        else {
                 phase = i%4;
        }
        out = 0;
        switch(phase){
                case 0:
                        out = FOCUS_APHASE;
                break;
                case 1:
                         out = FOCUS_APHASE+ FOCUS_BPHASE;
                break;
                case 2:
                         out = FOCUS_BPHASE;
                break;
                case 3:
                        out=0;
                break;
        }
   
        mask = exp1_gpiob & (~(FOCUS_APHASE + FOCUS_BPHASE));  //set before to the mask of phase a and b and the current output register
        exp1_gpiob = out + mask;
        DO_WRITE(addr1, REG_GPIOB, exp1_gpiob);
        std::this_thread::sleep_for(std::chrono::microseconds(STEPPER_SLEEP_TIME_MICRO));
   }

   focusLimit();
}


/**********************************************************/
//Functions relevant to Iris
/**********************************************************/

// Function to enable Iris
bool MotorDriver_CFE::enableIris() {
    if(use_gpio) {
        gpioExport(242);
        gpioExport(243);
        gpioDirection(242, 1);
        gpioDirection(243, 1);
        gpioSet(242, 1);
        gpioSet(243, 1);
    }
    else {
        exp0_gpioa |= IRIS_AENBL + IRIS_BENBL;
        DO_WRITE(addr0, REG_GPIOA, exp0_gpioa);
    }
    return true;
}

// Function to disable Iris
bool MotorDriver_CFE::disableIris()
{
    if(use_gpio) {
        gpioExport(242); // U7 pin A2 == IRIS AEN
        gpioExport(243); // U7 pin A3 == IRIS BEN
        gpioDirection(242, 1);
        gpioDirection(243, 1);
        gpioSet(242, 0);
        gpioSet(243, 0);
    }
    else {
        exp0_gpioa &= ~(IRIS_AENBL+IRIS_BENBL);
        DO_WRITE(addr0, REG_GPIOA, exp0_gpioa);
    }
    return true;
}


// Function for irisUp functionality
bool MotorDriver_CFE::irisUp(int steps)
{
    bool modified = false;
    if(iris_abs_location >= lens_motor_steps.iris.max) {
	    iris_abs_location = lens_motor_steps.iris.max;
        _mlog("Cannot irisup, reached maximum iris setting");
        modified = true;
    }
    else
    {
        enableIris();
        if(iris_abs_location + steps > lens_motor_steps.iris.max) 
        {
            steps = lens_motor_steps.iris.max - iris_abs_location;
	    }
	    iris(steps);
    	iris_abs_location += steps;
 	    disableIris();
        modified = true;
    }
    return modified;
}

// Function for irisDown functionality
bool MotorDriver_CFE::irisDown(int steps)
{
    bool modified = false;
    if(iris_abs_location <= lens_motor_steps.iris.min ) {
	    iris_abs_location = lens_motor_steps.iris.min;
        _mlog("Cannot irisdown, reached minimum iris setting");
        modified = true;
    }
    else 
    {
    	enableIris();
    	if((int)(iris_abs_location - steps) < (int)lens_motor_steps.iris.min) {
		steps = iris_abs_location;
	}
    	iris(0-steps);
    	iris_abs_location -= steps;
    	disableIris();
        modified = true;
    }
    return modified;
}


bool MotorDriver_CFE::irisHome()
{
    bool ret = false;
    int steps = lens_motor_steps.iris.max;
    //int steps = Configuration::iris_home_max_steps();
    //if(steps < LOWER_LIMIT || steps > IRIS_MAX_STEPS){
    //    steps = IRIS_MAX_STEPS;
    //    _mlog("Warning: iris steps cfg oob, setting steps to max size");
    //}
    int stepsize = 5;
    int counter = 0;
    irisDown(abs(stepsize));
    while (abs(counter) < steps)
    {
        irisUp(abs(stepsize));
        ostringstream oss;
        oss << "finding iris home " << counter << " " << stepsize;
        _mlog(oss.str());
        counter += stepsize;
    }
    if (abs(counter) <= steps) {
        ostringstream oss;
        oss << "Reached iris homing step limit of " << steps;
        _mlog(oss.str());
        iris_abs_location = 0;
        iris_init = true;
    } else {
        _mlog("Found iris limit");
        iris_abs_location = 0;
        iris_init = true;
        ret = true;
    }
    disableIris();
    return ret;
}

//Function to rotate motor for iris functionality as per the steps specified in the argument.
bool MotorDriver_CFE::iris(int steps)
{
    if (use_gpio) {
        char phase;
        phase = 0;
        gpioExport(246); // U7 pin A6 == IRIS_APHASE 
        gpioExport(241); // U7 pin A1 == IRIS_BPHASE
        gpioDirection(246, 1);
        gpioDirection(241, 1);
        for (int i=0; i < abs(steps); ++i) {
            if (steps > 0) {
                phase = 3 - (i%4);
            }
            else {
                phase = i%4;
            }
            switch(phase){
                case 0:
                    gpioSet(246, 1);
                    gpioSet(241, 0);
                    break;
                case 1:
                    gpioSet(246, 1);
                    gpioSet(241, 1);
                    break;
                case 2:
                    gpioSet(246, 0);
                    gpioSet(241, 1);
                    break;
                case 3:
                    gpioSet(246, 0);
                    gpioSet(241, 0);
                    break;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(STEPPER_SLEEP_TIME_MICRO*4));
        }
    }
    else {
        char mask;
        char out;
        char phase;
        for(int i = 0; i < abs(steps); ++i) {
            phase = 0;
            if (steps >0) {
                    phase = 3- (i%4);
            }
            else {
                     phase = i%4;
            }
            out = 0;
            switch(phase){
                    case 0:
                            out = IRIS_APHASE;
                    break;
                    case 1:
                             out = IRIS_APHASE+ IRIS_BPHASE;
                    break;
                    case 2:
                             out = IRIS_BPHASE;
                    break;
                    case 3:
                            out=0;
                    break;
            }

            mask = exp0_gpioa & (~(IRIS_APHASE + IRIS_BPHASE));  //set before to the mask of phase a and b and the current output register
            exp0_gpioa = out + mask;
            DO_WRITE(addr0, REG_GPIOA, exp0_gpioa);
            std::this_thread::sleep_for(std::chrono::microseconds(STEPPER_SLEEP_TIME_MICRO)*4); //150 pps
        } 
    }
    return true;
}


/**********************************************************/
//Functions relevant to IRCUT
/**********************************************************/

// Function for IRcut on functionality.
bool MotorDriver_CFE::ircutOn()
{
    if (pathExists("/sys/class/gpio/gpiochip240")) {
        use_gpio = true;
        gpioExport(247);
        gpioDirection(247, 1);
        gpioSet(247, 1);
    }
    if (use_gpio) {
        gpioExport(248);
        gpioExport(249);
        gpioDirection(248, 1);
        gpioDirection(249, 1);
        gpioSet(248, 0);
        gpioSet(249, 1);
    }
    else {
        exp0_gpiob |= IRCUT_A;
	    exp0_gpiob &= ~IRCUT_B;
	    DO_WRITE(addr0, REG_GPIOB, exp0_gpiob); 
    }
	return true;
	
}

// Function for IRcut off functionality.
bool MotorDriver_CFE::ircutOff()
{
    if (pathExists("/sys/class/gpio/gpiochip240")) {
        use_gpio = true;
        gpioExport(247);
        gpioDirection(247, 1);
        gpioSet(247, 1);
    }
    if (use_gpio) {
        gpioExport(248);
        gpioExport(249);
        gpioDirection(248, 1);
        gpioDirection(249, 1);
        gpioSet(248, 1);
        gpioSet(249, 0);
    }
    else {
        exp0_gpiob |= IRCUT_B;
        exp0_gpiob &= ~IRCUT_A;
        DO_WRITE(addr0, REG_GPIOB, exp0_gpiob);
    } 
	return true;
}


/**********************************************************/
//Functions relevant to GPIO
/**********************************************************/

// Exports the specified gpio pin
void MotorDriver_CFE::gpioExport(int pin)
{
    int fd;
    char buf[255];
    fd = open("/sys/class/gpio/export", O_WRONLY);
    sprintf(buf, "%d", pin);
    write(fd, buf, strlen(buf));
    close(fd);
}

// Sets the gpio value as per the specified pin and value
void MotorDriver_CFE::gpioSet(int pin, int value)
{
    int fd;
    char buf[255];
    sprintf(buf, "/sys/class/gpio/gpio%d/value", pin);
    fd = open(buf, O_WRONLY);
    sprintf(buf, "%d", value);
    write(fd, buf, 1);
    close(fd);
}

// Sets gpio direction as per the specified pin and direction
void MotorDriver_CFE::gpioDirection(int pin, int direction)
{
    int fd;
    char buf[255];
    sprintf(buf, "/sys/class/gpio/gpio%d/direction", pin);
    fd = open(buf, O_WRONLY);
    if (direction) {
        write(fd, "out", 3);
    }
    else {
        write(fd, "in", 2);
    }
    close(fd);
}

//Checks if the path exists as per the specified path in the parameter.
bool MotorDriver_CFE::pathExists(const std::string &s)
{
    struct stat buffer;
    return (stat (s.c_str(), &buffer) == 0);
}


// This function maps the input command to the correct function to execute for MotorDriver_CFE.
// The functions calls the MotorDriverBase::add_options since the commands are a subset of the ones in MotorDriver_CFE.
bool MotorDriver_CFE::add_options(po::options_description &desc)
{
    MotorDriverBase::add_options(desc);
    desc.add_options()
    ("zoomlocation", "prints zoom location")
    ("setzoom", po::value<unsigned int>(), "Sets zoom percent 0-100")
    ;
    desc.add_options()
    ("irislocation", "prints iris location")
    ("setiris", po::value<unsigned int>(), "Sets iris percent 0-100")
    ;
    return true;
}


// The function adds the keywords to be used as commands with their description.
// The functions calls the MotorDriverBase::add_options since the commands are a subset of the ones in MotorDriver_CFE.
bool MotorDriver_CFE::executeArgs(po::variables_map &vm, po::options_description &desc)
{
    MotorDriverBase::executeArgs(vm, desc);

    if (vm.count("zoomlocation")) {
        auto ret = zoomAbsoluteLocation();
        std::cout << "Zoom absolute location: " << ret << std::endl;
        return true;
    }
    else if (vm.count("setzoom")) 
    {
        DO_CMD(newZoomPercent(vm["setzoom"].as<unsigned int>()));
        return true;
    }
    else if (vm.count("irislocation")) {
        auto ret = irisAbsoluteLocation();
        std::cout << "Iris absolute location: " << ret << std::endl;
        return true;
    }
    else if (vm.count("setiris")) 
    {
        DO_CMD(newIrisPercent(vm["setiris"].as<unsigned int>()));
        return true;
    }
    return false;
}




bool MotorDriver_CFE::zoomRelative(int steps)
{
    bool ret = false;
    if (steps == 0) {
        return true;
    } else if (steps < 0) {
        ret = zoomDown(abs(steps));
    } else {
        ret = zoomUp(steps);
    }
    
    if (false == ret) {
        zoom_init = false;
        zoom_abs_location = -1;
    }
    return ret;
}


bool MotorDriver_CFE::irisRelative(int steps)
{
    bool ret = false;
    if (steps == 0) {
        return true;
    } else if (steps < 0) {
        ret = irisDown(abs(steps));
    } else {
        ret = irisUp(steps);
    }

    if (false == ret) {
        iris_init = false;
        iris_abs_location = -1;
    }
    return ret;
}


bool MotorDriver_CFE::focusRelative(int steps)
{
    bool ret = false;
    if (steps == 0) {
        return true;
    } else if (steps < 0) {
        ret = focusDown(abs(steps));
    } else {
        ret = focusUp(steps);
    }

    if (false == ret) {
        focus_init = false;
        focus_abs_location = -1;
    }
    return ret;
}



}

