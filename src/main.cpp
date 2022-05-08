
#include <sys/stat.h>
#include "global.h"
#include "gr/Router.h"

void signalHandler(int signum) {
    std::cout << "Signal (" << signum << ") received. Exiting...\n";

    std::exit(signum);
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    srand (time(NULL));

    std::cout << std::boolalpha;  // set std::boolalpha to std::cout
    
    log() << "-----------start-------------" << std::endl;

    //-------------------------- Input ---------------------------
    string netFile      = string(argv[1]);
    string output_path      = string(argv[2]);

    std::string preprefix = netFile.substr(0,netFile.find_last_of('.'));
    std::string prefix = preprefix.substr(0,preprefix.find_last_of('.'));
    std::string design = prefix.substr(prefix.find_last_of('/')+1);

    /*
    Setup log file 
    */
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

    /*
    Read input; Create database 
    */
    db::Database database;
    database.logger = &logger;
    database.designName = design;
    database.read(netFile);

    /*
    Run
    */
    utils::timer runtime;

    gr::Router router(&database);
    // router.Cluster();
    // router.KMeans();
    // router.BKMeans();
    // router.CKMeans();
    router.KMeansRefine();
    router.PatternRoute();
    router.ReRoute();

    router.write(output_path);
    // router.print_demand();

    // // Pattern Route
    // router.patter_route();
    // // Maze Route
    // router.break_ovfl();
    // double run_time = runtime.elapsed();
    // log() << "========== Execution time: " << run_time << " s ==========\n\n";

    // router.write(output_path);
    // //-------------------------- Verify -----------------------------
    // // Verify to log
    // string cmd = "../verifier/verify " + netFile + " " + output_path;
    // std::array<char, 128> buffer;
    // std::string log_out;
    // std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
    // if (!pipe) {
    //     throw std::runtime_error("popen() failed!");
    // }
    // while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    //     std::string data(buffer.data());
    //     log_out += data;
    // }

    // logger.info() << "\n\n\n" << log_out << endl;
    // log() << "Results logged to dir " << result_dir << endl;
    // log() << "Output written to " << output_path << endl << endl;

    return 0;
}