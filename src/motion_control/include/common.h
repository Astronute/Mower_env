#pragma once

#include <vector>
#include <memory>
#include <queue>
#include <chrono>
#include <iostream>

#include <google/protobuf/timestamp.pb.h>
#include "std_msgs/header.pb.h"
#include "nav_msgs/odometry.pb.h"
#include "geometry_msgs/pose.pb.h"
#include "geometry_msgs/twist.pb.h"
#include "pnc_msgs/point_vec.pb.h"
#include "pnc_msgs/planning_trajectory.pb.h"
#include "Eigen/Dense"

using SysTimePoint = std::chrono::time_point<std::chrono::system_clock>;
using SysTimeDuration = std::chrono::system_clock::duration;

namespace common {

	void quatToRPY(const geometry_msgs::Quaternion & quat, double & roll, double & pitch, double & yaw);

	void RPYToQuat(const double & roll, const double & pitch, const double & yaw, geometry_msgs::Quaternion & quat);

	void printVector(const std::vector<double> & vec);

	double nanosecToSec(const int64_t nanoseconds);

	SysTimePoint protoToSystime(const google::protobuf::Timestamp & stamp);

	google::protobuf::Timestamp systimeToProto(const SysTimePoint & time);

	double toSec(const SysTimePoint & time);

	double toSec(const std::chrono::system_clock::duration & duration);

	double toSec(const google::protobuf::Timestamp & stamp);

	double toMillisec(const SysTimePoint & time);

	double toMillisec(const std::chrono::system_clock::duration & duration);

	double linearEquation(double x1, double y1, double x2, double y2, double x);

	double linearEquationAngle(double x1, double y1, double x2, double y2, double x);

	double normalize_angle(double angle);

	double shortest_angular_distance(double angle_from, double angle_to);

	template<typename T>
	class fixedQueue{
	public:
		fixedQueue() : maxsize(5){}

		fixedQueue(size_t size) : maxsize(size){}

		void push(const T& value){
			std::unique_lock<std::mutex> lock(data_mutex);
			if(m_queue.size() >= maxsize){
				m_queue.pop();
			}
			m_queue.push(value);
		}

		void pop(){
			std::unique_lock<std::mutex> lock(data_mutex);
			if(m_queue.empty()){
				throw std::runtime_error("fixedQueue is empty");
			}
			m_queue.pop();
		}

		T back(){
			std::unique_lock<std::mutex> lock(data_mutex);
			if(m_queue.empty()){
				throw std::runtime_error("fixedQueue is empty");
			}
			return m_queue.back(); 
		}

		size_t size(){
			std::unique_lock<std::mutex> lock(data_mutex);
			return m_queue.size();
		}

		bool empty(){
			std::unique_lock<std::mutex> lock(data_mutex);
			return m_queue.empty();
		}

		void clear(){
			std::unique_lock<std::mutex> lock(data_mutex);
			std::queue<T> empty;
			m_queue.swap(empty);
		}

	private:
		std::queue<T> m_queue;
		size_t maxsize;
		mutable std::mutex data_mutex;
	};
}