#include <iostream>
#include <readline/readline.h>
#include <readline/history.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <boost/program_options.hpp>
#include <boost/iterator/transform_iterator.hpp>

#include "motordriver.hpp"

using namespace std;
using namespace boost;
using namespace ImageServer;


namespace po = boost::program_options;

static void lens_cout_log_handler(string output)
{
    cout << output << endl;
}

const char *convert_to_cstr(const std::string & s)
{
   return s.c_str(); 
}

void interactive(void) {

    //Creating a shared pointer for MotorDriverBase so that it get automatically deleted when its scope ends.
    std::shared_ptr<MotorDriverBase> m;
    
    //Deciding which motordriver to use by creating the respective class' object at runtime if the respective file path exists.
    
    m.reset(MotorDriver_factory::allocate(lens_cout_log_handler));
    
    
    po::options_description desc("Options");
    std::cout << "Verson 1.1.6: Type help for commands." << std::endl;
    //Runtime selection of help menu options over here for different lens model.
    m->add_options(desc);

    std::istringstream iss;
    std::vector<std::string> args;
    std::string token;
    po::variables_map vm;
    string input;
    char *buf;


    rl_bind_key('\t', rl_abort);
    while((buf = readline("\n> ")) != NULL)
    {
        //Exit the program if input is "quit"
        input = buf;
        if ("quit" == input) break;

        iss.clear();
        iss.str(input);
        args.clear();
        args.push_back("name");
        bool first = true;
        while (iss >> token) {
            // kind of ghetto, but add '--' to the first command, because we're just using boot::po
            // to do the parsing
            if(first)
            {
                token = "--" + token;
                first = false;
            }
            args.push_back(token);
        }
        auto beg = boost::make_transform_iterator(args.begin(), 
                convert_to_cstr);
        auto end = boost::make_transform_iterator(args.end(), 
                convert_to_cstr);

        const std::vector<const char*> vc { beg, end };
        vm.clear();
        try {
            po::store(po::parse_command_line(vc.size(), vc.data(), desc), vm);
            po::notify(vm);    
        } catch (std::exception &e) {
            std::cout << "Error: " << e.what() << std::endl;
            std::cout << desc << std::endl;
        }

        add_history(buf);

        //This function maps the input command keyword to the respective function.
        m->executeArgs(vm, desc);
    }
}


int main(int argc, const char* argv[])
{
    interactive();
    return 0;
}
