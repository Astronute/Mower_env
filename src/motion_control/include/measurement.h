#pragma once

#include <vector>
#include <memory>
#include <queue>
#include <chrono>
#include "Eigen/Dense"

using SysTimePoint = std::chrono::time_point<std::chrono::system_clock>;
using SysTimeDuration = std::chrono::system_clock::duration;

namespace CB {

	struct Measurement {
		Measurement() {}

		SysTimePoint time_;

		std::string topic_name_;

		double mahalanobis_thresh_;

		std::vector<bool> update_mask_;

		Eigen::VectorXd measurement_; // 测量结果
		
		Eigen::MatrixXd covariance_; // 协方差

		bool operator()(const std::shared_ptr<Measurement> & a, const std::shared_ptr<Measurement> & b){
			return (*this)(*(a.get()), *(b.get()));
		}

		bool operator()(const Measurement & a, const Measurement & b){
			return a.time_ > b.time_;
		}
	};
	using MeasurementPtr = std::shared_ptr<Measurement>;

	struct PathPoint{
		PathPoint() : x(0), y(0), z(0), yaw(0), kappa(0), s(0){}

		PathPoint(double x_, double y_, double z_, double yaw_, double kappa_, double s_){
			x = x_;
			y = y_;
			z = z_;
			yaw = yaw_;
			kappa = kappa_;
			s = s_;
		}

		double x, y, z;

		double yaw, kappa, s;
	};

	struct TrajPoint{
		TrajPoint() : path_point(0, 0, 0, 0, 0, 0), v(0), w(0), t(0){}

		TrajPoint(PathPoint path_point_, double v_, double w_, double t_){
			path_point = path_point_;
			v = v_;
			w = w_;
			t = t_;
		}

		PathPoint path_point;

		double v, w, t;
	};

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