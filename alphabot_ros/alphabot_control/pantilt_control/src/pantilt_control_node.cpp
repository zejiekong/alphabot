#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "vision_msgs/Detection2DArray.h"
#include "vision_msgs/Detection2D.h"
#include "vision_msgs/ObjectHypothesisWithPose.h"
#include <vector>
#include <signal.h>

ros::Publisher pub;
int threshold{20};

int pan_angle{90};
int tilt_angle{90};

void shutdownHandler(int sig)
{
    sensor_msgs::JointState cmd_msg;
    cmd_msg.position.push_back(90);
    cmd_msg.position.push_back(90);
    pub.publish(cmd_msg);
    ros::shutdown();
}

void detect_callback(const vision_msgs::Detection2DArrayConstPtr msg)
{
    int num_faces = msg->detections.size();
    int max_probability = 0;
    int max_num;
    for(int i = 0;i<num_faces;i++)
    {
        if(msg->detections[num_faces].results[0].score > max_probability)
        {
            max_probability = msg->detections[num_faces].results[0].score;
            max_num = i;
        }
    }
    int centre_x = msg->detections[max_num].bbox.center.x;
    int center_y = msg->detections[max_num].bbox.center.y;
    int error_x = centre_x - 205; //-ve turn right, +ve turn left
    int error_y = center_y - 154; //-ve move up, +ve move down
    sensor_msgs::JointState cmd_msg;
    if(error_x < -threshold && pan_angle!=160)
    {
        cmd_msg.position.push_back(++pan_angle);
        ROS_INFO("Left");
    }
    else if(error_x > threshold && pan_angle != 20)
    {
        cmd_msg.position.push_back(--pan_angle);
        ROS_INFO("Right");
    }
    else
    {
        cmd_msg.position.push_back(pan_angle);
    }
    if (error_y < -threshold && tilt_angle != 20)
    {
        cmd_msg.position.push_back(--tilt_angle);
        ROS_INFO("UP");
    }
    else if (error_y > threshold && tilt_angle != 160)
    {
        cmd_msg.position.push_back(++tilt_angle);
        ROS_INFO("Down");
    }
    else
    {
        cmd_msg.position.push_back(tilt_angle);
    }
    pub.publish(cmd_msg);
}

int main(int argc, char* argv[])
{
    ros::init(argc,argv,"pantilt_control_node");
    ros::NodeHandle nh;
    signal(SIGINT,shutdownHandler);
    if (nh.getParam("/threshold",threshold))
    {
        ROS_INFO("Successfully loaded configuration file. Threshold: %d",threshold);
    }
    else
    {
        ROS_INFO("Unsuccesful loading of configuration file. Default threshold: %d",threshold);
    }
    pub = nh.advertise<sensor_msgs::JointState>("/alphabot/servo_cmd",1);
    ros::Subscriber detect_sub = nh.subscribe("/alphabot/face_detection/bouding_boxes",1,detect_callback);
    ros::spin();
}
