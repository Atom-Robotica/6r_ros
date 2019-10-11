#include "iostream"
#include "unistd.h"
#include "ros/ros.h"
#include "std_msgs/UInt16.h"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "Servo_broadcaster");
    ros::NodeHandle nodeHandler;
    ros::Publisher msgPublisher = nodeHandler.advertise<std_msgs::UInt16>("servo", 10);
    int start_lim, end_lim;
    uint end_delay;
    double stride_duration, rate_freq;
    cout << "Enter start limit: ";
    cin >> start_lim;
    cout << "Enter end limit : ";
    cin >> end_lim;
    cout << "Enter duration for stride (sec): ";
    cin >> stride_duration;
    cout << "Enter end delay (msecs): ";
    cin >> end_delay;
    end_delay = end_delay * 1000;   // msec to usec
    std_msgs::UInt16 msgObj;
    rate_freq = (end_lim - start_lim)/(stride_duration);
    ros::Rate rateHandler(rate_freq);
    while (true) {
        for (int i = start_lim; i < end_lim; i ++) {
            msgObj.data = i;
            msgPublisher.publish(msgObj);
            rateHandler.sleep();
        }
        usleep(end_delay);
        for (int i = end_lim; i > start_lim; i --) {
            msgObj.data = i;
            msgPublisher.publish(msgObj);
            rateHandler.sleep();
        }
        usleep(end_delay);
        ros::spinOnce();
    }
    return 0;
}