#include "ros/ros.h"
#include "iostream"
using namespace std;

int main(int argc, char **argv)
{
    // Initialize node
    ros::init(argc, argv, "CPP_Echo_node");
    ros::NodeHandle nodeHandler;
    // Echo things
    cout << "Sample output through a C++ ROS Node." << endl;
    cout << "Press ENTER to exit" << endl;
    cin.get();
    return 0;
}

