#include "common.h"

namespace navicommon
{

    double nanosecToSec(const int64_t nanoseconds){
        return static_cast<double>(nanoseconds) * 1e-9;
    }

    SysTimePoint protoToSystime(const google::protobuf::Timestamp & stamp){
        return std::chrono::system_clock::time_point(
            std::chrono::seconds(stamp.seconds()) +
            std::chrono::nanoseconds(stamp.nanos())
        );
    }

    google::protobuf::Timestamp systimeToProto(const SysTimePoint & time){
        google::protobuf::Timestamp stamp;
        
        auto duration = time.time_since_epoch();
        stamp.set_seconds(std::chrono::duration_cast<std::chrono::seconds>(duration).count());
        stamp.set_nanos(std::chrono::duration_cast<std::chrono::nanoseconds>(duration % std::chrono::seconds(1)).count());

        return stamp;
    }

    double toSec(const SysTimePoint & time){
        return std::chrono::duration<double>(time.time_since_epoch()).count();
    }

    double toSec(const std::chrono::system_clock::duration & duration){
        return std::chrono::duration<double>(duration).count();
    }

    double toSec(const google::protobuf::Timestamp & stamp){
        return static_cast<double>(stamp.seconds()) + nanosecToSec(stamp.nanos());
    }
}