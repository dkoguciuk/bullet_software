#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>


int main(int argc,char** argv)
{
    //Initialize ROS and specify the name of node
    ros::init(argc,argv,"bullet_reset");
	
    //Handle to this process node
    ros::NodeHandle n("~");

    // Get param
    bool radians;
    n.param<bool>("float_angles", radians, true);

    // Declare publishers and subscribers
    ros::Publisher pub_joints_goal_positions;

    // Connect them
    if (radians)
    {
        pub_joints_goal_positions = n.advertise<std_msgs::Float32MultiArray>("/bullet_driver/present_joints_goal_position", 1);

        std_msgs::Float32MultiArray pos_array;
        pos_array.data.clear();
        for (int i=0; i<18; i++) pos_array.data.push_back((float)M_PI*(511.5-511.5)*300/1023/180);
        pub_joints_goal_positions.publish(pos_array);

        ros::Rate rate(10);
        for(int i=0; i<10; i++)
        {
            pub_joints_goal_positions.publish(pos_array);
            ros::spinOnce();
            rate.sleep();
        }
    } else
    {
        pub_joints_goal_positions = n.advertise<std_msgs::Int32MultiArray>("/bullet_driver/present_joints_goal_position", 1);

        std_msgs::Int32MultiArray pos_array;
        pos_array.data.clear();
        for (int i=0; i<18; i++) pos_array.data.push_back(511);
        pub_joints_goal_positions.publish(pos_array);

        ros::Rate rate(10);
        for(int i=0; i<10; i++)
        {
            pub_joints_goal_positions.publish(pos_array);
            ros::spinOnce();
            rate.sleep();
        }
    }

    // Return
	return 0;
}
