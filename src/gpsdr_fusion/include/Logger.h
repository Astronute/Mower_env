#ifndef LOGGER
#define LOGGER

#include <fstream>
#include <map>
#include <memory>
#include <atomic>
#include <mutex>
#include <thread>
#include <filesystem>
#include "Tools.h"
namespace clink
{
    class Logger
    {
    public:
        Logger(const std::string &fileName);
        void lock()
        {
            while (flag.test_and_set(std::memory_order_acquire))
            {
                std::this_thread::yield();
            }
        }
        void unlock()
        {
            flag.clear(std::memory_order_release);
        }
        template <typename T>
        std::fstream &operator<<(const T &message)
        {
            lock();
            std::shared_ptr<std::fstream> file = logFiles[logFileName];
            unlock();
            if (!file->is_open())
            {
                // std::cerr << "Error: Log file " << logFileName << " is not open." << std::endl;
                return *file;
            }

            *file << message;
            return *file;
        }
        // 新增：二进制写入接口
        void write(const char* data, size_t length)
        {
            lock();
            std::shared_ptr<std::fstream> file = logFiles[logFileName];
            unlock();

            if (file && file->is_open()) {
                file->write(data, length);
                file->flush();
            }
        }

    private:
        static std::map<std::string, std::shared_ptr<std::fstream>> logFiles;
        static std::atomic_flag flag;
        std::string logFileName;
    };
}

#endif // LOGGER