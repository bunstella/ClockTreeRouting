#include "log.h"

#include <iomanip>
#include <sstream>

#if defined(__unix__)
#include <sys/resource.h>
#include <unistd.h>
#elif defined(_WIN32)
#include <windows.h>
#include <psapi.h>
#endif

namespace utils {

timer::timer() { start(); }
void timer::start() { _start = clock::now(); }
double timer::elapsed() const { return std::chrono::duration<double>(clock::now() - _start).count(); }

timer tstamp;

std::ostream& operator<<(std::ostream& os, const timer& t) {
    std::ostringstream oss;  // seperate the impact of format
    oss << "[" << std::setprecision(3) << std::setw(8) << std::fixed << t.elapsed() << "] ";
    os << oss.str();
    return os;
}

double mem_use::get_current() {
#if defined(__unix__)
    long rss = 0L;
    FILE* fp = NULL;
    if ((fp = fopen("/proc/self/statm", "r")) == NULL) {
        return 0.0; /* Can't open? */
    }
    if (fscanf(fp, "%*s%ld", &rss) != 1) {
        fclose(fp);
        return 0.0; /* Can't read? */
    }
    fclose(fp);
    return rss * sysconf(_SC_PAGESIZE) / 1048576.0;
#elif defined(_WIN32)
    PROCESS_MEMORY_COUNTERS info;
    GetProcessMemoryInfo(GetCurrentProcess(), &info, sizeof(info));
    return info.WorkingSetSize / 1048576.0;
#else
    return 0.0;  // unknown
#endif
}

double mem_use::get_peak() {
#if defined(__unix__)
    struct rusage rusage;
    getrusage(RUSAGE_SELF, &rusage);
    return rusage.ru_maxrss / 1024.0;
#elif defined(_WIN32)
    PROCESS_MEMORY_COUNTERS info;
    GetProcessMemoryInfo(GetCurrentProcess(), &info, sizeof(info));
    return info.PeakWorkingSetSize / 1048576.0;
#else
    return 0.0;  // unknown
#endif
}

std::ostream& log(std::ostream& os) {
    os << tstamp;
    return os;
}

logger::logger(){
    std::time_t rawtime;
}

logger::logger(const std::string& file_dir, const std::string& file_name){
    std::time_t rawtime;
    std::tm* timeinfo;
    char buffer [80];

    std::time(&rawtime);
    timeinfo = std::localtime(&rawtime);
    std::strftime(buffer,80,"%Y_%m%d_%H%M%S",timeinfo);
    std::string stamp(buffer);
    std::string file_path_stamp = file_dir + stamp + "_" + file_name + ".log";
    fout = std::ofstream(file_path_stamp);
    
    fout << std::left << std::setw(20) << "Log file: ";
    fout << file_path_stamp << std::endl;
}

logger::~logger(){
    fout.close();
}

std::ofstream& logger::info(std::ostream& os) {
    // fout << tstamp;
    return fout;
}

}  // namespace utils