#include "iostream"
#include "chrono"

#include "zmq_subscriber.h"
#include <google/protobuf/timestamp.pb.h>
#include "std_msgs/header.pb.h"
#include "pnc_msgs/point_vec.pb.h"
#include "pnc_msgs/planning_trajectory.pb.h"
#include "pnc_msgs/motion_outsignal.pb.h"

#include "common.h"

	// // 控制器外部命令模式 default: UNDEFINED_CMD
	// enum ControllerMode{
	// 	SLOWLY_STOP_CMD,

	// 	RE_START_CMD,

	// 	LINE_MOVE_CMD,
	// 	ROTATE_MOVE_CMD,
	// 	CIRCLE_MOVE_CMD,

	// 	FORWARD_TO_POSE_CMD,
	// 	BACKWARD_TO_POSE_CMD,
	// 	ROTATE_TO_POSE_CMD,

	// 	UNDEFINED_CMD
	// };
int main(){

    std::cout << "rquest motion control node start" << std::endl;
    
    // test zmq client
    ZmqSubscriber zmq_subscriber;
    zmq_subscriber.initializeRequestClient("tcp://127.0.0.1:5588");

    pnc_msgs::MotionOutSignal motion_signal;
    motion_signal.mutable_header()->mutable_stamp()->CopyFrom(common::systimeToProto(std::chrono::system_clock::now()));
    motion_signal.set_id(2);
    motion_signal.set_name("signal_test");
    motion_signal.set_rsv_0(0.0);
    motion_signal.set_rsv_1(0.0);
    motion_signal.set_rsv_2(0.0);
    motion_signal.set_rsv_3(0.0);
    motion_signal.set_rsv_4(0.0);
    motion_signal.set_rsv_5(0.0);
    motion_signal.set_rsv_6(0.0);
    motion_signal.set_rsv_7(0.0);
    

    std::string serialized_data;
    motion_signal.SerializeToString(&serialized_data);
    
    zmq_subscriber.sendRequest(serialized_data);
    

    return 0;
}