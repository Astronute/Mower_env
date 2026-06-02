#pragma once

#include <vector>
#include <memory>
#include <queue>
#include <chrono>
#include <iostream>

#include <google/protobuf/timestamp.pb.h>

namespace navicommon
{

    using SysTimePoint = std::chrono::time_point<std::chrono::system_clock>;
    using SysTimeDuration = std::chrono::system_clock::duration;

    double nanosecToSec(const int64_t nanoseconds);

    SysTimePoint protoToSystime(const google::protobuf::Timestamp & stamp);

    google::protobuf::Timestamp systimeToProto(const SysTimePoint & time);

    double toSec(const SysTimePoint & time);

    double toSec(const std::chrono::system_clock::duration & duration);

    double toSec(const google::protobuf::Timestamp & stamp);
}

