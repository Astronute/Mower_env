#include "iostream"

#include "zmq_subscriber.h"



int main(){

    std::cout << "run mission node start" << std::endl;
    
    // test zmq client
    ZmqSubscriber zmq_subscriber;
    zmq_subscriber.initializeRequestClient("tcp://127.0.0.1:5544");
    std::string req = "/home/rpdzkj/Mower_env/mission_file/mission_test.json";
    zmq_subscriber.sendRequest(req);

    return 0;
}