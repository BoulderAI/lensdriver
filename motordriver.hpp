#pragma once

#include <stdint.h>
#include <iostream>
#include <memory>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <sys/stat.h>
#include <cmath>
namespace pt = boost::property_tree;

extern "C" {
    #include <i2c/smbus.h>
}

#define USE_SINCERE_IMX334_PATH     ("/data/production/lens/motor-driver/use-sincere-imx334")

namespace po = boost::program_options;

#define DO_CMD(cmd)   \
        {if (false == cmd) { std::cout << "FAILED" << std::endl; } else { std::cout << "OK" << std::endl; }}

namespace ImageServer
{


/*
* Consists of all the lens' focus and ircut functions. This class will act as the base class for inheritance purposes.
* This class is the parent class for both MotorDriver_imx334_SFOX334T1 and MotorDriver_CFE. 
* The shared pointer of this class will be used to call respective functions in the MotorDriver_imx334_SFOX334T1 and MotorDriver_CFE classes. 
*/
class MotorDriverBase {
public: 
    /**
    * Settings related to stepper motor configuration for lens related functions
    */
    struct step_settings
    {
        // the minimum number of steps for 0% setting
        unsigned int min;
        // the maximum number of steps for full range 100% setting
        unsigned int max;
    };

    /**
    * Lens settings for this lens/motor driver combination
    */
    struct lens_settings
    {
        struct step_settings focus;
        struct step_settings zoom;
        struct step_settings iris;
    };

    /**
    * The active lens configuration in percentage of total
    * steps.
    */
    struct lens_config
    {
        unsigned int zoom_percent;
        unsigned int focus_percent;
        unsigned int iris_percent;
    };

    virtual ~MotorDriverBase(){};

    bool init();

    // TODO: Replace steps by percentage once persistent storage is implemented.
    virtual bool focusUp(int steps) = 0;
    virtual bool focusDown(int steps) = 0;
    virtual int focusAbsoluteLocation() = 0;

    virtual bool ircutOn() = 0;
    virtual bool ircutOff() = 0;

    virtual bool executeArgs(po::variables_map &vm, po::options_description &desc);
    virtual bool add_options(po::options_description &desc);
    virtual bool irisUp(int steps) { return false; }
    virtual bool irisDown(int steps)  { return false; }
    virtual bool irisHome() { return false; }
    virtual int  irisAbsoluteLocation()  { return 0; }
    virtual bool zoomUp(int steps)  { return false; }
    virtual bool zoomDown(int steps) { return false; }
    virtual bool zoomHome()  { return false; }
    virtual int zoomAbsoluteLocation()  { return 0; }
    virtual bool printZoom()  { return false; }
    virtual bool zoomRelative(int steps)  { return false; }
    virtual bool focusRelative(int steps)  { return false; }
    virtual bool irisRelative(int steps)  { return false; }
    
    bool newFocusPercent( unsigned int percent );
    bool newZoomPercent( unsigned int percent );
    bool newIrisPercent( unsigned int percent );

    bool saveall_location_values(void);

protected:
    int zoom_abs_location;
    int focus_abs_location;
    int iris_abs_location;
    bool zoom_init;
    bool focus_init;
    bool iris_init;

    MotorDriverBase(const struct lens_settings &lensDefaults):
        zoom_init(false),
        zoom_abs_location(0),
        focus_init(false),
        focus_abs_location(0),
        iris_init(false),
        iris_abs_location(0)
    {
        lens_motor_steps = lensDefaults;
        lens_current.zoom_percent = 50;
        lens_current.focus_percent = 50;
        lens_current.iris_percent = 50;
    };

    virtual bool initImplementation() = 0;

    struct lens_settings lens_motor_steps;


private:
    // property tree for configuration items
    pt::ptree persistent_settings;
    pt::ptree production_settings;
    bool readSettings(pt::ptree &settings, const char *filepath);
    bool writeSettings(pt::ptree &settings, const char *filepath);
    struct lens_config lens_current;
    unsigned int calculateStep(const struct step_settings &step, unsigned int percent);
};


//Consists of all the lens' zoom functions.
class MotorDriver_zoom_interface {
public:
    MotorDriver_zoom_interface(){};
    virtual ~MotorDriver_zoom_interface(){};

    //TODO: Replace steps by percentage once persistent storage is implemented.
    virtual bool zoomUp(int steps) = 0;
    virtual bool zoomDown(int steps) = 0;
    virtual bool zoomHome() = 0;
    virtual int zoomAbsoluteLocation() = 0;
    virtual bool printZoom() = 0;

    virtual bool zoomRelative(int steps) = 0;
};


//Consists of all the lens' zoom functions.
class MotorDriver_iris_interface {
public:
    MotorDriver_iris_interface(){};
    virtual ~MotorDriver_iris_interface(){};

    //TODO: Replace steps by percentage once persistent storage is implemented.
    virtual bool irisUp(int steps) = 0;
    virtual bool irisDown(int steps) = 0;
    virtual bool irisHome() = 0;
    virtual int  irisAbsoluteLocation() = 0;

    virtual bool irisRelative(int steps) = 0;
};


//MotorDriver class for lens controlled by CFE (Example: Theia TL410P)
class MotorDriver_CFE : public MotorDriverBase, public MotorDriver_zoom_interface, public MotorDriver_iris_interface {
public:
    MotorDriver_CFE(bool doInit, boost::function < void(std::string) > log_callback);
    ~MotorDriver_CFE();

    bool enablePowerLines();

    // TODO: Replace steps by percentage once persistent storage is implemented.
    bool zoomUp(int steps);
    bool zoomDown(int steps);
    bool zoomHome();
    int  zoomAbsoluteLocation() { 
        if (lens_motor_steps.zoom.min + lens_motor_steps.zoom.max == 0) {return -1;} 
        else {        
        return ceil((double)(zoom_abs_location)*100/(unsigned int)(lens_motor_steps.zoom.min + lens_motor_steps.zoom.max)); }
        }

    bool focusUp(int steps);
    bool focusDown(int steps);
    bool focusHome();
    int  focusAbsoluteLocation() { 
        if (lens_motor_steps.focus.min + lens_motor_steps.focus.max == 0) {return -1;} 
        else {
            return ceil((double)(focus_abs_location)*100/(unsigned int)(lens_motor_steps.focus.min + lens_motor_steps.focus.max));} 
            }

    bool irisUp(int steps);
    bool irisDown(int steps);
    bool irisHome();
    int  irisAbsoluteLocation() { 
        if (lens_motor_steps.iris.min + lens_motor_steps.iris.max == 0) {return -1;}
        else {
            return ceil((double)(iris_abs_location)*100/(unsigned int)(lens_motor_steps.iris.min + lens_motor_steps.iris.max)); }
            }

    bool ircutOn();
    bool ircutOff();

    bool printZoom();
    bool printFocus();


    bool zoomRelative(int steps);
    bool focusRelative(int steps);
    bool irisRelative(int steps);

    bool executeArgs(po::variables_map &vm, po::options_description &desc);
    bool add_options(po::options_description &desc);

protected:
    static const  struct MotorDriverBase::lens_settings LensSettingsDefault_CFE_theia_tl410p;
    bool initImplementation();

private:
    enum class MotorDirection { UP, DOWN };
    bool initExpanders();
    bool writeReg(uint8_t addr, uint8_t regaddr, uint8_t data);
    bool readReg(uint8_t addr, uint8_t regaddr, uint8_t& res);
    bool printReg(uint8_t addr, uint8_t regaddr);

    bool enableZoom();
    bool disableZoom();
    bool zoom(int steps);
    bool zoomLimit();

    bool enableFocus();
    bool disableFocus();
    bool focus(int steps);
    bool focusLimit();

    bool enableIris();
    bool disableIris();
    bool iris(int steps);

    bool use_gpio = false;
    void gpioExport(int pin);
    void gpioSet(int pin, int value);
    void gpioDirection(int pin, int direction);
    bool pathExists(const std::string &s);

    int fd_;
    static int addr0, addr1, addr2;
    char exp0_gpioa, exp0_gpiob, exp1_gpioa, exp1_gpiob, exp2_gpioa, exp2_gpiob;
    bool iris_init;

    boost::function < void(std::string) > _log_callback;
};


//MotorDriver class for lens controlled by imx334 (Example: SFOX334T1)
class MotorDriver_imx334_SFOX334T1 : public MotorDriverBase {
public:
    MotorDriver_imx334_SFOX334T1(bool doInit, boost::function < void(std::string) > log_callback);
    ~MotorDriver_imx334_SFOX334T1();

    // TODO: Replace steps by percentage once persistent storage is implemented.
    bool focusUp(int steps);
    bool focusDown(int steps);
    int  focusAbsoluteLocation() { return focus_abs_location; }

    bool ircutOn();
    bool ircutOff();

protected:
    static const  struct MotorDriverBase::lens_settings LensSettingsDefault_imx334_SFOX334T1;
    bool initImplementation();
private:
    bool writeReg(uint8_t addr, uint16_t regaddr, uint16_t data);
    bool readReg(uint8_t addr, uint16_t regaddr, uint8_t& res);
    bool printReg(uint8_t addr, uint16_t regaddr);

    int fd_;
    static int addr3;
    boost::function < void(std::string) > _log_callback;
};




class MotorDriver_factory {
public: 

static class MotorDriverBase *allocate( boost::function < void(std::string) > log_callback){

    MotorDriverBase *m = NULL;
    struct stat buffer;
    if((stat(USE_SINCERE_IMX334_PATH, &buffer) == 0))
    {
        m = new MotorDriver_imx334_SFOX334T1(true, log_callback);
        log_callback("Path exists: Using IMX334 for motordriver");
    }
    else
    {
        m = new MotorDriver_CFE(true, log_callback);
        log_callback("Path does not exist: Using CFE for motordriver");
    }
    return m;
} 
    
};




}
