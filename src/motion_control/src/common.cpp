#include "common.h"

namespace common {

	void quatToRPY(const geometry_msgs::Quaternion & quat, double & roll, double & pitch, double & yaw){
		// roll (x-axis rotation)
		double sinr_cosp = 2 * (quat.w() * quat.x() + quat.y() * quat.z());
		double cosr_cosp = 1 - 2 * (quat.x() * quat.x() + quat.y() * quat.y());
		roll = std::atan2(sinr_cosp, cosr_cosp);
	
		// pitch (y-axis rotation)
		double sinp = 2 * (quat.w() * quat.y() - quat.z() * quat.x());
		if (std::abs(sinp) >= 1)
			pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
		else
			pitch = std::asin(sinp);
	
		// yaw (z-axis rotation)
		double siny_cosp = 2 * (quat.w() * quat.z() + quat.x() * quat.y());
		double cosy_cosp = 1 - 2 * (quat.y() * quat.y() + quat.z() * quat.z());
		yaw = std::atan2(siny_cosp, cosy_cosp);
	}

	void RPYToQuat(const double & roll, const double & pitch, const double & yaw, geometry_msgs::Quaternion & quat){
		double cy = cos(yaw * 0.5);
		double sy = sin(yaw * 0.5);
		double cp = cos(pitch * 0.5);
		double sp = sin(pitch * 0.5);
		double cr = cos(roll * 0.5);
		double sr = sin(roll * 0.5);
	
		quat.set_w(cy * cp * cr + sy * sp * sr);
		quat.set_x(cy * cp * sr - sy * sp * cr);
		quat.set_y(sy * cp * sr + cy * sp * cr);
		quat.set_z(sy * cp * cr - cy * sp * sr);
	}

	void printVector(const std::vector<double> & vec){
		if (vec.empty()) {
			std::cout << "empty param" << std::endl;
			return;
		}
		for (size_t i = 0; i < vec.size(); ++i) {
			std::cout << vec[i];
			if (i != vec.size() - 1) std::cout << ", ";
		}
		std::cout << "\n" << std::endl;
	}

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