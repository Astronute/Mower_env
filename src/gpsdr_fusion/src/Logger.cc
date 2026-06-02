#include "../include/Logger.h"

namespace clink
{
    std::map<std::string, std::shared_ptr<std::fstream>> Logger::logFiles;
    std::atomic_flag Logger::flag = ATOMIC_FLAG_INIT; // 原子标志，初始为未设置状态

    Logger::Logger(const std::string &fileName)
    {
        namespace fs = std::filesystem;

        std::string exeDir = clink::getExecutableDir();

// #ifdef SUNRISE_X3
//         std::string logFileRoot = "/userdata/applogs/yat_rtk2_algorithm_fusion";
// #else
        std::string logFileRoot = exeDir + "/../logs/yat_rtk2_algorithm_fusion";
// #endif
        

        // ===== 若目录不存在则创建（支持多级目录）=====
        if (!fs::exists(logFileRoot))
        {
            fs::create_directories(logFileRoot);
        }

        if (logFileRoot.back() != '/')
            logFileName = logFileRoot + "/" + fileName;
        else
            logFileName = logFileRoot + fileName;

        lock();
        if (logFiles.find(logFileName) == logFiles.end())
        {
            logFiles[logFileName] = 
                std::make_shared<std::fstream>(logFileName, std::ios::binary | std::ios::out);
        }
        unlock();
    }

}
