#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>

#include <mynteye/camera.h>

image_transport::Publisher gLeftImagePub;
image_transport::Publisher gRightImagePub;
ros::Publisher gImuPub;

double gStartROS;
std::uint32_t gStartIMU = 0;

ros::Time fromMyntStamp(std::uint32_t stamp)
{
	if(gStartIMU == 0)
	{
		gStartIMU = stamp;
		gStartROS = ros::Time::now().toSec();
	}
	return ros::Time(gStartROS + (stamp - gStartIMU) * 0.0001f);
}

void publishIMU(const mynteye::IMUData &imudata)
{
	sensor_msgs::Imu msg;

	msg.header.stamp = fromMyntStamp(imudata.time);
	msg.header.frame_id = "base_link";

	msg.linear_acceleration.x = imudata.accel_x * 9.8;
	msg.linear_acceleration.y = imudata.accel_y * 9.8;
	msg.linear_acceleration.z = imudata.accel_z * 9.8;

	msg.linear_acceleration_covariance[0] = 0.04;
	msg.linear_acceleration_covariance[1] = 0;
	msg.linear_acceleration_covariance[2] = 0;

	msg.linear_acceleration_covariance[3] = 0;
	msg.linear_acceleration_covariance[4] = 0.04;
	msg.linear_acceleration_covariance[5] = 0;

	msg.linear_acceleration_covariance[6] = 0;
	msg.linear_acceleration_covariance[7] = 0;
	msg.linear_acceleration_covariance[8] = 0.04;

	msg.angular_velocity.x = imudata.gyro_x / 57.2956;
	msg.angular_velocity.y = imudata.gyro_y / 57.2956;
	msg.angular_velocity.z = imudata.gyro_z / 57.2956;

	msg.angular_velocity_covariance[0] = 0.02;
	msg.angular_velocity_covariance[1] = 0;
	msg.angular_velocity_covariance[2] = 0;

	msg.angular_velocity_covariance[3] = 0;
	msg.angular_velocity_covariance[4] = 0.02;
	msg.angular_velocity_covariance[5] = 0;

	msg.angular_velocity_covariance[6] = 0;
	msg.angular_velocity_covariance[7] = 0;
	msg.angular_velocity_covariance[8] = 0.02;

	msg.orientation.x = 0;
	msg.orientation.y = 0;
	msg.orientation.z = 0;
	msg.orientation.w = 1;

	gImuPub.publish(msg);
}

int main(int argc, char **argv)
{
	// ROS stuff
    ros::init(argc, argv, "mynt_eye");
    ros::NodeHandle n;
	
	image_transport::ImageTransport transport(n);
	gLeftImagePub = transport.advertise("left_image", 1);
	gRightImagePub = transport.advertise("right_image", 1);

	gImuPub = n.advertise<sensor_msgs::Imu>("imu", 1, true);

	// Mynt-Eye stuff
	mynteye::InitParameters init;

	mynteye::Camera cam;
	cam.Open(init);
	if (!cam.IsOpened())
	{
		ROS_ERROR("Could not open Mynt-Eye!");
		return 1;
	}

	while(n.ok())
	{
		if (cam.Grab() == mynteye::ErrorCode::SUCCESS)
		{
			std_msgs::Header header;
			cv::Mat leftImg,rightImg;

			// Get left image
			if(cam.RetrieveImage(leftImg, mynteye::View::VIEW_LEFT_UNRECTIFIED) == mynteye::ErrorCode::SUCCESS)
			{
//				std::cout << "Got left image at " << cam.GetTimestamp() << std::endl;
				header.stamp = fromMyntStamp(cam.GetTimestamp());
				header.frame_id = "left_camera";
				gLeftImagePub.publish(cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, leftImg).toImageMsg());
			}else
			{
				ROS_WARN("Failed to retreive left image.");
			}

			// Get right image
			if(cam.RetrieveImage(rightImg, mynteye::View::VIEW_RIGHT_UNRECTIFIED) == mynteye::ErrorCode::SUCCESS)
			{
//				std::cout << "Got right image at " << cam.GetTimestamp() << std::endl;
				header.stamp = fromMyntStamp(cam.GetTimestamp());
				header.frame_id = "right_camera";
				gRightImagePub.publish(cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, rightImg).toImageMsg());
			}else
			{
				ROS_WARN("Failed to retreive right image");
			}

			// Get IMU
			std::vector<mynteye::IMUData> imu_samples;
			if(cam.RetrieveIMUData(imu_samples) == mynteye::ErrorCode::SUCCESS)
			{
//				std::cout << "Got " << imu_samples.size() << " IMU samples:" << std::endl;
				for(auto sample = imu_samples.begin(); sample < imu_samples.end(); sample++)
				{
					double time = (double)sample->time / 10000.0;
//					std::cout << time << " (" << sample->time << ")" << std::endl;
					publishIMU(*sample);
				}
			}
		}else
		{
			ROS_ERROR("Could not grab camera image.");
		}
	}

	cam.Close();
	return 0;
}
