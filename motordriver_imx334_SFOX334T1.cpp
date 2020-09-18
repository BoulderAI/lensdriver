// System includes
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <chrono>
#include <thread>

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

// Address of the IMX334 chip on the I2C bus
int MotorDriver_imx334_SFOX334T1::addr3 = 0x60;

#define LSB_MASK(x)     ((uint8_t)(x))                  //Extract least significant byte using this mask
#define MSB_MASK(x)     ((uint8_t)(((x) >> 8)))         //Extract most significant byte using this mask.

//As per the imx334 datasheet
#define FOCUS_FORWARD   (0x4000)                        
#define FOCUS_REVERSE   (0x6000)                        
#define REG_IRCUT       (0x0112)
#define REG_MOTOR_CTRL  (0x0101)
#define IRCUT_ON        (0x0002)
#define IRCUT_OFF       (0x0003)

const  struct MotorDriverBase::lens_settings MotorDriver_imx334_SFOX334T1::LensSettingsDefault_imx334_SFOX334T1 =
{
    // Found experimentally as we don't have a datasheet for this
    .focus =  { .min = 0, .max= 2300 }
};


#define DO_WRITE(addr,reg,data) \
	{if (false == writeReg(addr,reg,data)) { ostringstream oss; oss << "Write failed for addr: " << hex << addr << " reg: " << hex << reg << dec; _mlog(oss.str()); return false;}}

MotorDriver_imx334_SFOX334T1::MotorDriver_imx334_SFOX334T1(bool doInit, boost::function < void(std::string) > log_callback):
    MotorDriverBase(MotorDriver_imx334_SFOX334T1::LensSettingsDefault_imx334_SFOX334T1),
    fd_(-1)
{        
    if (true == doInit) {
        init();
    }
}

MotorDriver_imx334_SFOX334T1::~MotorDriver_imx334_SFOX334T1()
{}

// Initialization function for I2C with sensor module.
bool MotorDriver_imx334_SFOX334T1::initImplementation()
{
    _mlog("Initializing i2c-2 device");
    std::string devicename("/dev/i2c-2");
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

    if (-1 == fd_) {
        _mlog("i2c-2 device not initialized");
        return false;
    }

    if (ioctl(fd_, I2C_SLAVE, addr3) < 0) {
        ostringstream oss;
        oss << "Unable to access slave: " << addr3;
        _mlog(oss.str());
        return false;
    }
    return true;
}


/*
* @brief: Write function for I2C communicating with sensor module.
* @param1: Slave address
* @param2: Register address to write to.
* @param3: Data to write to the register address.
*/
bool MotorDriver_imx334_SFOX334T1::writeReg(uint8_t addr, uint16_t regaddr, uint16_t data)
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
    uint8_t buf[4];
    buf[0] = MSB_MASK(regaddr);
    buf[1] = LSB_MASK(regaddr);
    buf[2] = MSB_MASK(data);
    buf[3] = LSB_MASK(data);

    if (write(fd_, buf, 4) != 4) {
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
* TODO: readReg does not work. Need to work on this, module does not respond as as per the datasheet
*/
bool MotorDriver_imx334_SFOX334T1::readReg(uint8_t addr, uint16_t regaddr, uint8_t& res)
{
    if (ioctl(fd_, I2C_SLAVE, addr ) < 0) {
        ostringstream oss;
        oss << "Unable to access slave: " << addr;
        _mlog(oss.str());
        return false;
    }

    uint8_t buf[2];
    buf[0] = MSB_MASK(regaddr);
    buf[1] = LSB_MASK(regaddr);

    if (write(fd_, buf, 2) != 2) {
        _mlog("Unable to write data");
        return false;
    }

    uint8_t ret = 0;
    ret = read(fd_, buf, 4);
    ostringstream oss;
    oss << "Return Value" << hex << std::setw(2) << std::setfill('0') << (unsigned)ret << " MSB 0x" << std::setw(2) << std::setfill('0') << (unsigned)buf[0] << " LSB 0x" << std::setw(2) << std::setfill('0') << (unsigned)buf[1] << " MSB 0x" << std::setw(2) << std::setfill('0') << (unsigned)buf[2] << " LSB 0x" << std::setw(2) << std::setfill('0') << (unsigned)buf[3] << dec;
    _mlog(oss.str());

    return true;
}

/* 
* @brief: Prints the data read from a register address
* @param1: Slave address
* @param2: Register address to read from.
*/
bool MotorDriver_imx334_SFOX334T1::printReg(uint8_t addr, uint16_t regaddr)
{
    uint8_t res = 0x00;
    bool ret = readReg(addr, regaddr, res);
    ostringstream oss;
    oss << "0x" << std::hex << res << std::dec;
    _mlog(oss.str());
    return ret;
}


/*****************************************************/
//Functions relevant to FOCUS
/*****************************************************/

/* 
* @brief: Rotates the lens' motor for focus functionality in forward direction with the specified parameter steps
* @param1: integer value of steps to rotate
*/
bool MotorDriver_imx334_SFOX334T1::focusUp(int steps)
{	
    if(focus_abs_location >= lens_motor_steps.focus.max ) {
        focus_abs_location = lens_motor_steps.focus.max;
        _mlog("Cannot focusup, reached maximum focus setting");
    }
    else {
        if( (focus_abs_location + steps) > lens_motor_steps.focus.max )
        {
            steps = lens_motor_steps.focus.max - focus_abs_location;
        }
        DO_WRITE(addr3, REG_MOTOR_CTRL, steps | FOCUS_FORWARD);
        focus_abs_location += steps;
    }
    return true;
}


/* 
* @brief: Rotates the lens' motor for focus functionality in reverse direction with the specified parameter steps
* @param1: integer value of steps to rotate
*/
bool MotorDriver_imx334_SFOX334T1::focusDown(int steps)
{
    if(focus_abs_location <= lens_motor_steps.focus.min ) {
	    focus_abs_location = lens_motor_steps.focus.min ;
        _mlog("Cannot zoomdown, reached minimum focus setting");
    }
    else {
    	if(focus_abs_location < (lens_motor_steps.focus.min + steps) ){
	        steps = focus_abs_location - lens_motor_steps.focus.min;
    	}
        DO_WRITE(addr3, REG_MOTOR_CTRL, steps | FOCUS_REVERSE);
        focus_abs_location -= steps;
    }
    return true;
}


/**********************************************************/
//Functions relevant to IRCUT
/**********************************************************/

// Function for IRcut on functionality.
bool MotorDriver_imx334_SFOX334T1::ircutOn()
{
    DO_WRITE(addr3, REG_IRCUT, IRCUT_ON);
	return true;
}

// Function for IRcut off functionality.
bool MotorDriver_imx334_SFOX334T1::ircutOff()
{
    DO_WRITE(addr3, REG_IRCUT, IRCUT_OFF);
	return true;
}

}

