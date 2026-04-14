#include <iostream>
#include <chrono>
#include <string>

#include "pnc_msgs/point_vec.pb.h"
#include "pnc_msgs/planning_trajectory.pb.h"
#include "zmq_publisher.h"
#include "zmq_subscriber.h"

void message_callback(const std::string& message, const std::string& topic){
    if(topic == "local_traj"){
        pnc_msgs::PlanningTrajectory local_traj;
        if(local_traj.ParseFromArray(message.data(), message.size())){
            std::cout << local_traj.header().stamp().seconds() << "frame_id: " << local_traj.header().frame_id() << std::endl;
            std::vector<pnc_msgs::PointVec8f> path(local_traj.trajectory().begin(), local_traj.trajectory().end());

            for(int i=0; i<path.size(); ++i){
                std::cout << "pont " << i << " x: " << path[i].x() << " y: " << path[i].y() << std::endl;
            }
        }
        else{
            std::cout << "process failed" << std::endl;
        }
    }
    else{
        std::cout << "topic: " << topic << " process callback not defined" << std::endl;
    }
}

int test_zmq_main(){
    std::cout << "fuck world" << std::endl;

    // init publisher
    ZmqPublisher local_traj_pub;
    local_traj_pub.initialize("tcp://*:5557");

    // init subscriber
    SubscriberConfig sub_cfg;
    sub_cfg.address = "tcp://localhost:5557";
    sub_cfg.topics = {"local_traj"};
    std::vector<SubscriberConfig> cfgs = {sub_cfg};
    ZmqSubscriber local_traj_sub;
    local_traj_sub.initialize(cfgs);
    local_traj_sub.setMessageCallback(message_callback);

    local_traj_sub.start();

    while(true){
        auto now = std::chrono::system_clock::now();
        int64_t seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
        int64_t nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count() % 1000000000;

        pnc_msgs::PlanningTrajectory traj;
        traj.mutable_header()->mutable_stamp()->set_seconds(seconds);
        traj.mutable_header()->mutable_stamp()->set_nanos(nanos);
        traj.mutable_header()->set_frame_id("odom");
        for(int i=0; i<7; i++){
            pnc_msgs::PointVec8f* p = traj.add_trajectory();
            p->set_x(i);
            p->set_y(i);
            p->set_yaw(i);
            p->set_kappa(i);
            p->set_w(i);
            p->set_v(i);
            p->set_t(i);
            p->set_s(i);
        }
        std::string serialized_data;
        traj.SerializeToString(&serialized_data);
        local_traj_pub.publishMessage("local_traj", serialized_data);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }


    return 0;
}