#include "iostream"
#include "chrono"

#include "zmq_subscriber.h"
#include <google/protobuf/timestamp.pb.h>
#include "std_msgs/header.pb.h"
#include "pnc_msgs/point_vec.pb.h"
#include "pnc_msgs/planning_trajectory.pb.h"
#include "pnc_msgs/motion_outsignal.pb.h"

#include "common.h"


int main(){

    std::cout << "rquest motion control node start" << std::endl;
    
    // test zmq client
    ZmqSubscriber zmq_subscriber;
    zmq_subscriber.initializeRequestClient("tcp://127.0.0.1:5588");

    pnc_msgs::MotionOutSignal motion_signal;
    motion_signal.mutable_header()->mutable_stamp()->CopyFrom(common::systimeToProto(std::chrono::system_clock::now()));
    motion_signal.set_id(2);
    motion_signal.set_name("signal_test");

    std::string serialized_data;
    motion_signal.SerializeToString(&serialized_data);
    
    zmq_subscriber.sendRequest(serialized_data);
    

    return 0;
}