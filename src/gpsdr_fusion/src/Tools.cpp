#include "../include/Tools.h"

namespace clink
{
    std::string getExecutableDir()
    {
        char result[PATH_MAX];
        ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
        std::string path = std::string(result, (count > 0) ? count : 0);

        // 去掉可执行文件名，只保留目录
        size_t lastSlash = path.find_last_of('/');
        return path.substr(0, lastSlash);
    }
}
