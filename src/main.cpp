#include <iostream>
#include <ros/ros.h>
#include <time.h>
#include <string>
#include <sstream>

// keyboard input tool
#include "keyinput.h"
#include "camlidar_logger.h"

// Get current data/time, format is yyyy-mm-dd.hh:mm:ss
const std::string currentDateTime(){
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about data/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d_%H_%M_%S", &tstruct);

    return buf;
}

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "camlidar_logger_node");
    ros::NodeHandle nh("~");

    int n_cams   = -1;
    int n_lidars = -1;
    string dir;    
    ros::param::get("~n_cameras", n_cams);
    ros::param::get("~n_lidars", n_lidars);
    ros::param::get("~directory", dir);
    
    
    // Ground control system class
    
    stringstream ss1;
    ss1 << dir << currentDateTime() << "/";
    string save_dir = ss1.str();
    cout << "save directory:[" << save_dir << "]\n";


    // user input manual.
    string user_manual;
    stringstream ss;
    ss << "\n==============================================\n|" 
    << "  Press a key..." 
    << "\n|    i: get data & start all algorithms"
    << "\n|    f: continuous shot mode"
    << "\n|    s: query & save one scene" 
    << "\n|    q: cease the program"
    << "\n|    m: camera parameter configuration mode"
    << "\n|  Select an input: \n";
    user_manual = ss.str();
    cout << user_manual;

    string cam_configure_manual;
    ss.clear();
    ss.flush();

    ss << "\n |\n L\n";

    CamlidarLogger* camlidar_logger = new CamlidarLogger(nh, n_cams, n_lidars, save_dir);

    int cnt = 0;
    while(ros::ok())
    {
        // no need 'spinOnce()' for all loop!!
        int c = getch(); // call my own non-blocking input function
        if(c == 's') {
            cout << "\n\n[Operation]: snapshot & save the current scene.\n";
        }
        else if(c == 'q') {
            cout << "\n\n[Operation]: quit program\n\n";
            break;
        }
        else if((c != 0)) {
            cout << ": Un-identified command...\n";
            cout << user_manual;
        }       
        ros::spinOnce();
    }
    ROS_INFO_STREAM("End of the program.\n");
    return -1;
}
