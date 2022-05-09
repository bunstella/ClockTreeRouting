
#include <sys/stat.h>
#include "global.h"
#include "gr/Router.h"

void signalHandler(int signum) {
    std::cout << "Signal (" << signum << ") received. Exiting...\n";

    std::exit(signum);
}

inline const char * const BoolToString(bool b){
            return b ? "true" : "false";}

void runProgram(const boost::program_options::variables_map& vm) {
    //-------------------------- Input ---------------------------
    string netFile  = vm.at("input").as<std::string>();
    string output_path = vm.at("output").as<std::string>();
    bool verify = vm.at("verify").as<bool>();
    bool plot = vm.at("plot").as<bool>();

    std::string prefix = netFile.substr(0,netFile.find_last_of('.'));
    std::string design = prefix.substr(prefix.find_last_of('/')+1);
    std::string root = prefix.substr(0,prefix.find_last_of('/'));

    /* Setup log file  */
    const std::string& result = "results";
    const std::string& result_dir = "../" + result + "/";
    struct stat sb;
    stat(result_dir.c_str(), &sb);
    if (!S_ISDIR(sb.st_mode)) {
        mkdir(result_dir.c_str(), 0777);
    }
    utils::logger logger(result_dir, design);
    logger.info() << left << setw(20) << "Ouput path: ";
    logger.info() << output_path << endl;

    /* Read input; Create database  */
    db::Database database;
    database.logger = &logger;
    database.designName = design;
    database.read(netFile);

    /* Run */
    utils::timer runtime;
    gr::Router router(&database);

    /* Cluster */
    if (!router.KMeansRefine()) {
        router.Cluster();
    }
    // router.Cluster();
    // router.KMeans();
    // router.BKMeans();
    // router.CKMeans();
    // router.KMeansRefine();

    /* Route */
    router.PatternRoute();
    router.ReRoute();
    router.write(output_path);

    /*-------------------------- Verify -----------------------------*/
    if (verify) {
        /* Verify to log */
        string cmd = "python ../eval.py --input " + netFile + " --output " + output_path + " --fig " 
                                + root + "/../results --plot " + BoolToString(plot);
        log() << "Running " << cmd << endl;
        std::array<char, 128> buffer;
        std::string log_out;
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
        if (!pipe) {
            throw std::runtime_error("popen() failed!");
        }
        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
            std::string data(buffer.data());
            log_out += data;
        }
        logger.info() << "\n\n\n" << log_out << endl;
    }

    log() << "Results logged to dir " << result_dir << endl;
    log() << "Output written to " << output_path << endl << endl;
}

int main(int argc, char* argv[]) {
    printlog("------------------------------------------------------------------------------");
    printlog("                    CENG 4120 Project: ClockTree(no) Routing                  ");
    printlog("------------------------------------------------------------------------------");
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    srand (time(NULL));
    std::cout << std::boolalpha;  // set std::boolalpha to std::cout
    try {
        using namespace boost::program_options;
        options_description desc{"Options"};
        // clang-format off
        desc.add_options()
                ("input", value<std::string>()->required(), "")
                ("output", value<std::string>()->required(), "")
                ("verify", value<bool>()->default_value(true), "")
                ("plot", value<bool>()->default_value(false), "")
                ;
        // clang-format on
        variables_map vm;
        store(command_line_parser(argc, argv)
                  .options(desc)
                  .style(command_line_style::unix_style | command_line_style::allow_long_disguise)
                  .run(),
              vm);
        notify(vm);
        for (const auto& option : desc.options()) {
            if (vm.count(option->long_name())) {
                std::string name = option->description().empty() ? option->long_name() : option->description();
                log() << std::left << std::setw(18) << name << ": ";
                const auto& value = vm.at(option->long_name()).value();
                if (auto v = boost::any_cast<double>(&value)) {
                    std::cout << *v;
                } else if (auto v = boost::any_cast<int>(&value)) {
                    std::cout << *v;
                } else if (auto v = boost::any_cast<std::string>(&value)) {
                    std::cout << *v;
                } else if (auto v = boost::any_cast<bool>(&value)) {
                    std::cout << *v;
                } else {
                    std::cout << "unresolved type";
                }
                std::cout << std::endl;
            }
        }
        log() << std::endl;
        // Entry
        runProgram(vm);
    } catch (const boost::program_options::error& e) {
        printlog(e.what());
    }

    return 0;
}