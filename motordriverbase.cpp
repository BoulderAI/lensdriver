// System includes
#include <boost/program_options.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>
#include "motordriver.hpp"


using namespace std;
namespace po = boost::program_options;



static void _mlog(std::string in) { std::cout << in << std::endl;  };

// This path would consist of the default values related to zoom, focus and iris.
// i.e their respective min and max steps, and default percent values for a clear image.
#define PRODUCTION_DEFAULT_SETTINGS_JSON_PATH           "/data/production/lens/default.json"

// This path would store the user modified percent values of zoom, focus and iris
// so that rebooting the device doesn't affect the lens configuration.
#define PERSISTENT_SETTINGS_JSON_PATH                   "/data/bai-cm/template-data/lens/config.json"

// This path would determine if the device is being replugged.
// imx334 resets its motor on replugging the device everytime. This path would act a marker to determine 
// if the device was replugged.
// This is redundant if the motor is being controlled by CFE since CFE doesn't reset the motor at all on replugging the device.
#define VOLATILE_PERSISTENT_SETTINGS_JSON_PATH          "/var/volatile/lens/motordriver-init"

namespace ImageServer
{

// This function maps the input command to the correct function to execute for MotorDriver_imx334_SFOX334T1.
// This function has been defined in class MotorDriverBase instead of MotorDriver_imx334_SFOX334T1 
// since we do not want any direct relation between MotorDriver_imx334_SFOX334T1 and MotorDriver_CFE.
// We let inheritance handle the call to correct functions for MotorDriver_CFE and MotorDriver_imx334_SFOX334T1.
bool MotorDriverBase::executeArgs(po::variables_map &vm, po::options_description &desc)
{
    if (vm.count("help")) {
            // remove the dashes since we're using boost::po to parse this interactively
            ostringstream remove_dashes_oss;
            remove_dashes_oss << desc;
            string remove_dashes = remove_dashes_oss.str();
            boost::replace_all(remove_dashes, "--", "");
            std::cout << remove_dashes << endl;
            return true;
        }
        else if (vm.count("focuslocation")) {
            auto ret = focusAbsoluteLocation();
            std::cout << "Focus absolute location: " << ret << std::endl;
            return true;
        }
        else if (vm.count("ircuton")) {
            DO_CMD(ircutOn());
            return true;
        }
        else if (vm.count("ircutoff")) {
            DO_CMD(ircutOff());
            return true;
        }
        else if (vm.count("setfocus")) {
            DO_CMD(newFocusPercent(vm["setfocus"].as<unsigned int>()));
            return true;
        }
    return false;
}


// The function adds the keywords to be used as commands with their description for MotorDriver_imx334_SFOX334T1.
// This function has been defined in class MotorDriverBase instead of MotorDriver_imx334_SFOX334T1 
// since we do not want any direct relation between MotorDriver_imx334_SFOX334T1 and MotorDriver_CFE.
// We let inheritance handle the call to correct functions for MotorDriver_CFE and MotorDriver_imx334_SFOX334T1.
bool MotorDriverBase::add_options(po::options_description &desc)
{
        desc.add_options()
        ("help", "print help message")
        ("quit", "leave this program")
        ;
        desc.add_options()
        ("focuslocation", "prints focus location")
        ("setfocus", po::value<unsigned int>(), "Sets focus percent 0-100")
        ;
        desc.add_options()
        ("ircuton", "flips the ir-cut filter on")
        ("ircutoff", "flips the ir-cut filter off")
        ;
        return true;
}


/**
* Read settings in the json file at @param filepath into the ptree at @param settings
*/
bool MotorDriverBase::readSettings(pt::ptree &settings, const char *filepath)
{
    bool success = false;
    /**
    * See http://www.cochoy.fr/boost-property-tree/
    */
    try {
        // Load the json file in this ptree
        pt::read_json(filepath, settings);
        success = true;
    }
    catch (const pt::json_parser::json_parser_error& e)
    {
        ostringstream oss;
        oss << "Invalid JSON read from  " << filepath << " no motor driver settings loaded: " << e.what();
        _mlog(oss.str());
    }
    if( success ) {
        ostringstream oss;
        oss << "Read settings from " << filepath;
        _mlog(oss.str());
    }
    return success;
}

/**
* Write settings from the ptree at @param settings into the file at @param filepath
*/
bool MotorDriverBase::writeSettings(pt::ptree &settings, const char *filepath)
{
    bool success = false;
    ofstream settingsFile (filepath);
    if (settingsFile.is_open()) {
        try 
        {
            pt::write_json(settingsFile, settings);
            success = true;
        }
        catch (const pt::json_parser::json_parser_error& e)
        {
            ostringstream oss;
            oss << "Failed to write JSON to " << filepath << " with error " << e.what();
            _mlog(oss.str());
        }
        settingsFile.close();
    } else {
        ostringstream oss;
        oss << "Failed to open " << filepath << ", could not update persistent settings";
    }

    return success;
}

/**
* Load from settings tree at @param ptree a value with type @param type
* located in json structure based on structure member name in @param structure_member
* Use the current value of @param structure_member as the default if the value
* could not be loaded
*/
#define LOAD_JSON(ptree,type,structure_member) \
    structure_member = ptree.get<type>(#structure_member,structure_member);

/**
* Write to settings tree at @param ptree a value with type @param type
* located in json structure based on structure member name in @param structure_member
* and write these values out to persistent storage as well.
*/
#define WRITE_JSON(ptree,structure_member) \
    ptree.put(#structure_member,structure_member); \
    writeSettings(ptree,PERSISTENT_SETTINGS_JSON_PATH);



bool MotorDriverBase::init()
{
    bool success = initImplementation();
    
    if (readSettings(production_settings,PRODUCTION_DEFAULT_SETTINGS_JSON_PATH))
    {
        /**
        * See http://www.cochoy.fr/boost-property-tree/
        */
        LOAD_JSON(production_settings,unsigned int,lens_motor_steps.zoom.min);
        LOAD_JSON(production_settings,unsigned int,lens_motor_steps.zoom.max);
        LOAD_JSON(production_settings,unsigned int,lens_motor_steps.focus.min);
        LOAD_JSON(production_settings,unsigned int,lens_motor_steps.focus.max);
        LOAD_JSON(production_settings,unsigned int,lens_motor_steps.iris.min);
        LOAD_JSON(production_settings,unsigned int,lens_motor_steps.iris.max);

        
        _mlog("Printing Min/Max steps");
        ostringstream oss;
        oss << "Zoom min: " << lens_motor_steps.zoom.min; 
        _mlog(oss.str()); oss.str("");
        oss << "Zoom max: " << lens_motor_steps.zoom.max; 
        _mlog(oss.str()); oss.str("");
        oss << "Focus min: " << lens_motor_steps.focus.min; 
        _mlog(oss.str()); oss.str("");
        oss << "Focus max: " << lens_motor_steps.focus.max; 
        _mlog(oss.str()); oss.str("");
        oss << "Iris min: " << lens_motor_steps.iris.min; 
        _mlog(oss.str()); oss.str("");
        oss << "Iris max: " << lens_motor_steps.iris.max; 
        _mlog(oss.str()); oss.str("");
        //Inserting blank line
        _mlog("");


        LOAD_JSON(persistent_settings,unsigned int,lens_current.zoom_percent);
        LOAD_JSON(persistent_settings,unsigned int,lens_current.focus_percent);
        LOAD_JSON(persistent_settings,unsigned int,lens_current.iris_percent);

        _mlog("Default values being loaded: ");
        oss << "Zoom percent: " << lens_current.zoom_percent; 
        _mlog(oss.str()); oss.str("");
        oss << "Focus percent: " << lens_current.focus_percent; 
        _mlog(oss.str()); oss.str("");
        oss << "Iris percent: " << lens_current.iris_percent; 
        _mlog(oss.str()); oss.str("");
        //Inserting blank line
        _mlog("");
    }
    else
    {
        _mlog("Failed to read setting from default.json, using default hardcoded values");
    }



    //Checking for persistent Storage Values
    if (readSettings(persistent_settings,PERSISTENT_SETTINGS_JSON_PATH))
    {
        _mlog("Overwriting hard-coded/default.json values using the below values");
        LOAD_JSON(persistent_settings,unsigned int,lens_current.zoom_percent);
        LOAD_JSON(persistent_settings,unsigned int,lens_current.focus_percent);
        LOAD_JSON(persistent_settings,unsigned int,lens_current.iris_percent);

        _mlog("Persistent Storage Current Values");
        ostringstream oss;
        oss << "Zoom percent: " << lens_current.zoom_percent; 
        _mlog(oss.str()); oss.str("");
        oss << "Focus percent: " << lens_current.focus_percent; 
        _mlog(oss.str()); oss.str("");
        oss << "Iris percent: " << lens_current.iris_percent; 
        _mlog(oss.str()); oss.str("");

        //Inserting blank line
        _mlog("");
    }
    else
    {
        _mlog("Failed to read setting from config.json, using default.json");
    }

    saveall_location_values();

    // Setting lens to latest values from persistent storage
    if(!newZoomPercent(lens_current.zoom_percent))
    {
        _mlog("Failed to adjust zoom using configuration file");
    }
    if (!newFocusPercent(lens_current.focus_percent))
    {
        _mlog("Failed to adjust Focus using configuration file");
    }
    
    if(!newIrisPercent(lens_current.iris_percent))
    {
        _mlog("Failed to adjust Iris using configuration file");
    }

    //Inserting blank line
    _mlog("");
    return success;
}

unsigned int MotorDriverBase::calculateStep(const struct step_settings &step, unsigned int percent)
{
    unsigned int rtnstep = step.min;
    if( percent != 0 ) {
        if( percent > 100 )  {
            rtnstep = step.max;
        }
        rtnstep = (((unsigned long)(step.min + step.max))*percent)/100;
    }
    return rtnstep;
}

bool MotorDriverBase::saveall_location_values(void)
{
    // struct stat volatile_dir;
    // if (stat(VOLATILE_PERSISTENT_SETTINGS_JSON_PATH, &volatile_dir) == 0 && S_ISDIR(volatile_dir.st_mode))
    // {
        zoom_abs_location = calculateStep(lens_motor_steps.zoom,lens_current.zoom_percent);
        focus_abs_location = calculateStep(lens_motor_steps.focus,lens_current.focus_percent);
        iris_abs_location = calculateStep(lens_motor_steps.iris,lens_current.iris_percent);


        _mlog("Motor Step Values for Corresponding Saved Percentage Values");
        ostringstream oss;
        oss << "Zoom_abs_location = " << zoom_abs_location;
        _mlog(oss.str()); oss.str("");
        oss << "Focus_abs_location = " << focus_abs_location;
        _mlog(oss.str()); oss.str(""); 
        oss << "Iris_abs_location = " << iris_abs_location;
        _mlog(oss.str()); oss.str("");
    // }
    // else
    // {
    //     // Creating the empty directory
    //     // See https://www.cplusplus.com/forum/windows/85516/#msg458826
    //     boost::filesystem::path dir(VOLATILE_PERSISTENT_SETTINGS_JSON_PATH);
	//     if(boost::filesystem::create_directories(dir)) {
	// 	    std::cout << "Success" << "\n";
	//     }
    // }
    
    //Inserting blank line
    _mlog("");
    return true;
}

bool MotorDriverBase::newFocusPercent( unsigned int percent )
{
    unsigned int newStep = calculateStep(lens_motor_steps.focus,percent);
    bool modified = false;
    if( newStep != focus_abs_location ) {
        if( newStep > focus_abs_location ) {
            modified=focusUp(newStep - focus_abs_location);
        } else {
            modified=focusDown(focus_abs_location-newStep);
        }
        if( modified ) {
            lens_current.focus_percent = percent;
            WRITE_JSON(persistent_settings,lens_current.focus_percent);
        }
    }
    else
    {
        std::cout << "Focus already at " << percent << " percent" << std::endl;
        modified = true;
    }
    return modified;
}



bool MotorDriverBase::newZoomPercent( unsigned int percent )
{
    unsigned int newStep = calculateStep(lens_motor_steps.zoom,percent);
    bool modified = false;
    if( newStep != zoom_abs_location ) {
        if( newStep > zoom_abs_location ) {
            modified = zoomUp(newStep - zoom_abs_location);
        } else {
            modified = zoomDown(zoom_abs_location-newStep);
        }
        if( modified ) {
            lens_current.zoom_percent = percent;
            WRITE_JSON(persistent_settings,lens_current.zoom_percent);
        }
    }
    else
    {
        std::cout << "Zoom already at " << percent << " percent" << std::endl;
        modified = true;
    }
    return modified;
}



bool MotorDriverBase::newIrisPercent( unsigned int percent )
{
    unsigned int newStep = calculateStep(lens_motor_steps.iris,percent);
    bool modified = false;
    if( newStep != iris_abs_location ) {
        if( newStep > iris_abs_location ) {
            modified = irisUp(newStep - iris_abs_location);
        } else {
            modified = irisDown(iris_abs_location - newStep);
        }
        if( modified ) {
            lens_current.iris_percent = percent;
            WRITE_JSON(persistent_settings,lens_current.iris_percent);
        }
    }
    else
    {
        std::cout << "Iris already at " << percent << " percent" << std::endl;
        modified = true;
    }
    return modified;
}

} // namespace ImageServer