#include <ros/ros.h>
#include "bullet_robot.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

BulletRobot *bullet_robot;

void intArrayCallback(const std_msgs::Int32MultiArray::ConstPtr& array)
{
    if (array->data.size() != 18)
    {
        std::cerr << "Wrong size!" << std::endl;
        exit(-1);
    }

    uint16_t pos[18];
    for (int i=0; i<array->data.size(); ++i) pos[i] = (uint16_t) array->data.at(i);
    bullet_robot->writeAllPosition(pos);
}
void floatArrayCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
    if (array->data.size() != 18)
    {
        std::cerr << "Wrong size!" << std::endl;
        exit(-1);
    }

    uint16_t pos[18];
    for (int i=0; i<array->data.size(); ++i) pos[i] = round(array->data.at(i)*180/M_PI*1023/300+ 511.5);
    // *180/M_PI for conversion from radians to degrees, *1023/300 because value 1023 represents servo 300 degrees position
    bullet_robot->writeAllPosition(pos);
}

int main(int argc,char** argv)
{
    // Connect to the robot
    bullet_robot = new BulletRobot;
    bullet_robot->changeBulletSTMMode(BULLET_MODE_OPERATIONAL);

    //Initialize ROS and specify the name of node
    ros::init(argc,argv,"bullet_driver");
	
    //Handle to this process node
    ros::NodeHandle n;

    // Get param
    bool radians;
    n.param<bool>("float_angles", radians, true);

    // Declare publishers and subscribers
    ros::Publisher pub_joints_present_positions, pub_joints_speed, pub_joints_load;
    ros::Subscriber sub_joints_goal_position;

    // Connect them
    if (radians)
    {
        pub_joints_present_positions = n.advertise<std_msgs::Float32MultiArray>("joint_states", 1);
        sub_joints_goal_position = n.subscribe("joint_goals", 1, floatArrayCallback);
    } else
    {
        pub_joints_present_positions = n.advertise<std_msgs::Int32MultiArray>("joint_states", 1);
        sub_joints_goal_position = n.subscribe("joint_goals", 1, intArrayCallback);
    }

    //Define node frequency
    ros::Rate loop_rate(100);
    uint16_t old_pos[18], old_speed[18], old_torque[18];
    uint16_t new_pos[18], new_speed[18], new_torque[18];

	//While ROS is ok
	while(ros::ok())
	{
        // Read all params
        BulletStatus stat = bullet_robot->readAllPositionSpeedTorque(new_pos, new_speed, new_torque);
        for (int i=0; i<18; ++i)
        {
            if (new_pos[i] > 1022 || new_pos[i]<1) new_pos[i] = old_pos[i];
            if (new_speed[i] > 1022 || new_speed[i]<1) new_speed[i] = old_speed[i];
            if (new_torque[i] > 1022 || new_torque[i]<1) new_torque[i] = old_torque[i];
        }
        memcpy(old_pos, new_pos, 18*sizeof(uint16_t));
        memcpy(old_speed, new_speed, 18*sizeof(uint16_t));
        memcpy(old_torque, new_torque, 18*sizeof(uint16_t));

        // Publish them
        if (radians)
        {
            std_msgs::Float32MultiArray pos_array;
            pos_array.data.clear();
            for (int i=0; i<18; i++) pos_array.data.push_back((float)M_PI*(new_pos[i]-511.5)*300/1023/180);
            pub_joints_present_positions.publish(pos_array);
        } else
        {
            std_msgs::Int32MultiArray pos_array;
            pos_array.data.clear();
            for (int i=0; i<18; i++) pos_array.data.push_back(new_pos[i]);
            pub_joints_present_positions.publish(pos_array);
        }

		//For receiving purposes
		ros::spinOnce();

        //Sleep for the remaining time
        loop_rate.sleep();
	}

    // Disconnect
    delete bullet_robot;

    // Return
    return 0;
}
