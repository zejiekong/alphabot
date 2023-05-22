#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <detection_msgs/BoundingBoxes.h>
#include <detection_msgs/BoundingBox.h>
#include <vector>

ros::Publisher pub;

int pan_angle = 90;
int tilt_angle = 90;
void state_callback(const sensor_msgs::JointStateConstPtr msg)
{
    pan_angle = msg->position[0];
    tilt_angle = msg->position[1];
}

void detect_callback(const detection_msgs::BoundingBoxesConstPtr msg)
{
    int num_faces = msg->bounding_boxes.size();
    int max_probability = 0;
    int max_num;
    for(int i = 0;i<num_faces;i++)
    {
        if(msg->bounding_boxes[num_faces].probability > max_probability)
        {
            max_probability = msg->bounding_boxes[num_faces].probability;
            max_num = i;
        }
    }
    int centre_x = (msg->bounding_boxes[max_num].xmax + msg->bounding_boxes[max_num].xmin)/2;
    int center_y = (msg->bounding_boxes[max_num].ymax + msg->bounding_boxes[max_num].ymin)/2;
    int error_x = centre_x - 205; //-ve turn right, +ve turn left
    int error_y = center_y - 154; //-ve move up, +ve move down
    sensor_msgs::JointState cmd_msg;
    if(error_x < -20)
    {
        if(pan_angle!=170)
        {
            //cmd_msg.position.push_back(pan_angle+1);
            ROS_INFO("Right");
        }
    }
    else if(error_x > 20)
    {
        if(pan_angle != 10)
        {
            //cmd_msg.position.push_back(pan_angle-1);
            ROS_INFO("Left");
        }
    }
    if (error_y < -20)
    {
        if(tilt_angle != 10)
        {
            //cmd_msg.position.push_back(tilt_angle-1);
            ROS_INFO("UP");
        }
    }
    else if (error_y > 20)
    {
        if(tilt_angle != 170)
        {
            //cmd_msg.position.push_back(tilt_angle+1);
            ROS_INFO("Down");
        }
    }
    pub.publish(cmd_msg);
}

int main(int argc, char* argv[])
{
    ros::init(argc,argv,"pantilt_control_node");
    ros::NodeHandle n;
    pub = n.advertise<sensor_msgs::JointState>("/alphabot/servo_cmd",10);
    ros::Subscriber state_sub = n.subscribe("/alphabot/servo_state",1,state_callback);
    ros::Subscriber detect_sub = n.subscribe("/alphabot/face_detection",1,detect_callback);
    ros::spin();
}
