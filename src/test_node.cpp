#include <tf/transform_datatypes.h>

#include "ros/ros.h"

#include "sensor_msgs/Imu.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Vector3Stamped.h"

ros::Subscriber imuSub;
ros::Publisher imuPub, magPub, eulerPub;

void getEulerRad(double q[4], double &x, double &y, double &z) {
    x = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi
    y = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
    z = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
}

// Convert orientation queternion from the filter to euler angles
void onImuMsg(const sensor_msgs::Imu::ConstPtr &msg) {
    geometry_msgs::Vector3Stamped eulerMsg;
    eulerMsg.header = msg->header;
    double q[4] = {msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z};
    getEulerRad(q, eulerMsg.vector.x, eulerMsg.vector.y, eulerMsg.vector.z);
    eulerMsg.vector.x *= 180/M_PI;
    eulerMsg.vector.y *= 180/M_PI;
    eulerMsg.vector.z *= 180/M_PI;
    eulerPub.publish(eulerMsg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_node");
    
    ros::NodeHandle nh;
    
    imuPub = nh.advertise<sensor_msgs::Imu>("/imu/data_raw", 10);
    magPub = nh.advertise<geometry_msgs::Vector3Stamped>("/imu/mag", 10);
    eulerPub = nh.advertise<geometry_msgs::Vector3Stamped>("/imu/euler", 10);
    
    imuSub = nh.subscribe("/imu/data", 10, &onImuMsg);
    
    ros::Rate r(250);
    while (ros::ok())
    {
        ros::Time stamp = ros::Time::now();
        
        //Publish raw imu data (copied from phidgets spatial 3/3/3 imu)
        sensor_msgs::Imu imuMsg;
        imuMsg.header.stamp = stamp;
        imuMsg.header.frame_id = "imu";

        imuMsg.angular_velocity.x = 0.0;
        imuMsg.angular_velocity.y = 0.0;
        imuMsg.angular_velocity.z = 0.0;

        imuMsg.angular_velocity_covariance[0] = 1.2184696791468346e-07;
        imuMsg.angular_velocity_covariance[4] = 1.2184696791468346e-07;
        imuMsg.angular_velocity_covariance[8] = 1.2184696791468346e-07;


        imuMsg.linear_acceleration.x = 0.0;
        imuMsg.linear_acceleration.y = 0.0;
        imuMsg.linear_acceleration.z = 9.81;

        imuMsg.linear_acceleration_covariance[0] = 8.66124974095918e-06;
        imuMsg.linear_acceleration_covariance[4] = 8.66124974095918e-06;
        imuMsg.linear_acceleration_covariance[8] = 8.66124974095918e-06;

        imuPub.publish(imuMsg); 
        
        //Publish magnetometer data
        geometry_msgs::Vector3Stamped magMsg;
        magMsg.header.stamp = stamp;
        magMsg.header.frame_id = "imu";
//        magMsg.vector.x = -0.420;
//        magMsg.vector.y = +0.302;
//        magMsg.vector.z = -0.835;
        magMsg.vector.x = 0.04848;
        magMsg.vector.y = 0.05151;
        magMsg.vector.z = 0.33936;
        magPub.publish(magMsg);   
    
        ros::spinOnce();
        r.sleep();
    }
}
