#ifndef _CAMLIDAR_LOGGER_H_
#define _CAMLIDAR_LOGGER_H_
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <fstream> // for lidar pcd files

#include <ros/ros.h>
#include <Eigen/Dense>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// for subscribe
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

string dtos(double x)
{
	stringstream s;
	s << setprecision(6) << fixed << x;
	return s.str();
};

string itos(double x)
{
	stringstream s;
	s << x;
	return s.str();
};

class CamlidarLogger{
public:
    CamlidarLogger(ros::NodeHandle& nh, int n_cams, int n_lidars, vector<string>& image_names, const string& save_dir);
    ~CamlidarLogger();


    void pointcloud2tobuffers(const sensor_msgs::PointCloud2ConstPtr& msg_lidar, const int& id);
    void pointcloud2tobuffersVelodyneVLP16(const sensor_msgs::PointCloud2ConstPtr& msg_lidar, const int& id);
    void pointcloud2tobuffersOusterOS1Gen264(const sensor_msgs::PointCloud2ConstPtr& msg_lidar, const int& id);
    void saveLidarData(const std::string& file_name, const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_lidar);
    void saveLidarDataRingTime(const std::string& file_name, const int& id);
    void saveAllData();

    // pointer to data
    cv::Mat& getBufImage(const int& id){ return *(buf_imgs_+id);};
    pcl::PointCloud<pcl::PointXYZI>::Ptr& getBufLidar(const int& id){return *(buf_lidars_+id);};

    inline int getNumCams(){return n_cams_;};
    inline int getNumLidars(){return n_lidars_;};
private:
    // node handler
    ros::NodeHandle nh_;

    // subscribers
    image_transport::ImageTransport it_;
    vector<image_transport::Subscriber> subs_imgs_; // from mvBlueCOUGAR-X cameras
    ros::Subscriber sub_timestamp_; // from arduino.
    vector<ros::Subscriber> subs_lidars_; // from Velodyne lidars

    // topic names
    vector<string> topicnames_imgs_;
    vector<string> topicnames_lidars_;

    // state variables
    int n_cams_; // numbering rule(cams) 0,1) cabin left,right, 2,3) boom frontal,rear
    int n_lidars_;// numbering rule(lidars)- 0) cabin, 1) boom

    // transmittion flags.
    bool* flag_imgs_; // 'true' when image data is received.
    bool* flag_lidars_; // 'true' when lidar data is received.

    // data container (buffer)
    cv::Mat* buf_imgs_; // Images from mvBlueCOUGAR-X cameras.
    pcl::PointCloud<pcl::PointXYZI>::Ptr* buf_lidars_; // point clouds (w/ intensity) from Velodyne

    vector<float*> buf_lidars_x;
    vector<float*> buf_lidars_y;
    vector<float*> buf_lidars_z;
    vector<float*> buf_lidars_intensity;
    vector<unsigned short*> buf_lidars_ring;
    vector<float*> buf_lidars_time;
    vector<int> buf_lidars_npoints;

    // private methods
    void initializeAllFlags();
    void callbackImage(const sensor_msgs::ImageConstPtr& msg, const int& id);
    void callbackLidar(const sensor_msgs::PointCloud2ConstPtr& msg_lidar, const int& id);

    string save_dir_;
    int current_seq_;

};

CamlidarLogger::CamlidarLogger(ros::NodeHandle& nh, 
int n_cams, int n_lidars, 
vector<string>& image_names,
const string& save_dir)
: nh_(nh), it_(nh_), n_cams_(n_cams), n_lidars_(n_lidars),save_dir_(save_dir)
{
    // default.
    flag_lidars_ = nullptr;
    flag_imgs_   = nullptr;
    buf_imgs_    = nullptr;

    // initialize image container & subscribers.
    if(n_cams_ > 0){
        buf_imgs_  = new cv::Mat[n_cams_];
        flag_imgs_ = new bool[n_cams_];
        for(int i = 0; i < n_cams_; i++) {
            flag_imgs_[i] = false;
            //string name_temp = "/" + itos(i) + "/image_raw";
            string name_temp = image_names[i];
            topicnames_imgs_.push_back(name_temp);
            subs_imgs_.push_back(it_.subscribe(topicnames_imgs_[i], 1, boost::bind(&CamlidarLogger::callbackImage, this, _1, i)));
        }
    }
    else {
        buf_imgs_  = nullptr;
        flag_imgs_ = nullptr;
    }

    // initialize lidar container & subscribers.
    if(n_lidars_ > 0){
        buf_lidars_ = new pcl::PointCloud<pcl::PointXYZI>::Ptr[n_lidars_];
        flag_lidars_ = new bool[n_lidars_];
        for(int i = 0; i < n_lidars_; i++){
            flag_lidars_[i] = false;
            *(buf_lidars_ + i) = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            buf_lidars_x.push_back(new float[400000]);
            buf_lidars_y.push_back(new float[400000]);
            buf_lidars_z.push_back(new float[400000]);
            buf_lidars_intensity.push_back(new float[400000]);
            buf_lidars_ring.push_back(new unsigned short[400000]);
            buf_lidars_time.push_back(new float[400000]);
            buf_lidars_npoints.push_back(0);

            string name_temp = "/lidar" + itos(i) + "/velodyne_points";
            topicnames_lidars_.push_back(name_temp);
            subs_lidars_.push_back(nh_.subscribe<sensor_msgs::PointCloud2>(topicnames_lidars_[i], 1, boost::bind(&CamlidarLogger::callbackLidar, this, _1, i)));
        }
    }
    else{
        buf_lidars_  = nullptr;
        flag_lidars_ = nullptr;
    }

    // generate save folder
    std::string folder_create_command;
    folder_create_command = "sudo rm -rf " + save_dir_;
	system(folder_create_command.c_str());
    folder_create_command = "mkdir " + save_dir_;
	system(folder_create_command.c_str());

    // make image saving directories
    for(int i = 0; i< n_cams_; i++) {
        folder_create_command = "mkdir " + save_dir_ + "cam" + itos(i) + "/";
	    system(folder_create_command.c_str());
    }
    // make lidar data saving directories
    for(int i = 0; i < n_lidars_; i++) {
        folder_create_command = "mkdir " + save_dir_ + "lidar" + itos(i) + "/";
	    system(folder_create_command.c_str());
    }

    // save association
    string file_name = save_dir_ + "/association.txt";
    std::ofstream output_file(file_name, std::ios::trunc);
    output_file.precision(6);
    output_file.setf(std::ios_base::fixed, std::ios_base::floatfield);
    if(output_file.is_open()){
        output_file << "#seq ";
        for(int i = 0; i < n_cams_; i++) output_file << "cam" << i << " ";
        for(int i = 0; i < n_lidars_; i++) output_file << "lidar" << i <<" ";
        output_file << "\n";
    }
}

CamlidarLogger::~CamlidarLogger(){
    // ! all allocation needs to be freed.
    if( buf_imgs_ != nullptr ) delete[] buf_imgs_;
    if( flag_imgs_ != nullptr ) delete[] flag_imgs_;

    if( buf_lidars_ != nullptr) delete[] buf_lidars_;
    if( flag_lidars_ != nullptr) delete[] flag_lidars_;
    for(int i = 0; i < n_lidars_; i++){
	delete[] buf_lidars_x[i];
	delete[] buf_lidars_y[i];
	delete[] buf_lidars_z[i];
	delete[] buf_lidars_intensity[i];
	delete[] buf_lidars_ring[i];
	delete[] buf_lidars_time[i];
    }
}


void CamlidarLogger::initializeAllFlags(){
    for(int i = 0; i < n_cams_; i++) flag_imgs_[i] = false;
    for(int i = 0; i < n_lidars_; i++){
        flag_lidars_[i] = false;
        buf_lidars_npoints[i] = 0;
    }
};


void CamlidarLogger::pointcloud2tobuffers(const sensor_msgs::PointCloud2ConstPtr& msg_lidar, const int& id){
    // get width and height of 2D point cloud data
    if(msg_lidar->height > 1){ // ouster 64. (height == 64)
	    this->pointcloud2tobuffersOusterOS1Gen264(msg_lidar,id);
    }
    else{ // velodyne VLP 16. (height == 1)
	    this->pointcloud2tobuffersVelodyneVLP16(msg_lidar, id);
    }
}

void CamlidarLogger::pointcloud2tobuffersVelodyneVLP16(const sensor_msgs::PointCloud2ConstPtr& msg_lidar, const int& id){
    // get width and height of 2D point cloud data
    buf_lidars_npoints[id] = msg_lidar->width;

    for(int i = 0; i < msg_lidar->width; i++) {
       int arrayPosX = i*msg_lidar->point_step + msg_lidar->fields[0].offset; // X has an offset of 0
       int arrayPosY = i*msg_lidar->point_step + msg_lidar->fields[1].offset; // Y has an offset of 4
       int arrayPosZ = i*msg_lidar->point_step + msg_lidar->fields[2].offset; // Z has an offset of 8

       int ind_intensity = i*msg_lidar->point_step + msg_lidar->fields[3].offset; // 12
       int ind_ring = i*msg_lidar->point_step + msg_lidar->fields[4].offset; // 16
       int ind_time = i*msg_lidar->point_step + msg_lidar->fields[5].offset; // 18

       memcpy(buf_lidars_x[id]+i,         &msg_lidar->data[arrayPosX],     sizeof(float));
       memcpy(buf_lidars_y[id]+i,         &msg_lidar->data[arrayPosY],     sizeof(float));
       memcpy(buf_lidars_z[id]+i,         &msg_lidar->data[arrayPosZ],     sizeof(float));
       memcpy(buf_lidars_intensity[id]+i, &msg_lidar->data[ind_intensity], sizeof(float));
       memcpy(buf_lidars_ring[id]+i,      &msg_lidar->data[ind_ring],      sizeof(unsigned short));
       memcpy(buf_lidars_time[id]+i,      &msg_lidar->data[ind_time],      sizeof(float));
    }
}

void CamlidarLogger::pointcloud2tobuffersOusterOS1Gen264(const sensor_msgs::PointCloud2ConstPtr& msg_lidar, const int& id){
    // get width and height of 2D point cloud data
    buf_lidars_npoints[id] = msg_lidar->width*msg_lidar->height;

    int n_pts_channel = msg_lidar->width;
    int n_channels    = msg_lidar->height;

    cout << "width: "      << n_pts_channel << endl;
    cout << "height: "     << n_channels    << endl;
    cout << "point step: " << msg_lidar->point_step << endl;
    cout << "row step: "   << msg_lidar->row_step   << endl;

    for(int i = 0; i < 9; i++)
        cout << msg_lidar->fields[i].name << " / " << msg_lidar->fields[i].offset<<endl;
    

    for(int ch = 0; ch < msg_lidar->height; ch++) {
        int offset_channel = ch*msg_lidar->row_step; // 98304 = 64 channels * 2048 points (2048 mode)

        for(int i = 0; i < msg_lidar->width; i++) {
            int offset_i = offset_channel + i*msg_lidar->point_step;
            int ind_x = offset_i + msg_lidar->fields[0].offset; // X has an offset of 0 (sz: 4)
            int ind_y = offset_i + msg_lidar->fields[1].offset; // Y has an offset of 4 (sz: 4)
            int ind_z = offset_i + msg_lidar->fields[2].offset; // Z has an offset of 8 (sz: 4)
            int ind_intensity = offset_i + msg_lidar->fields[3].offset; // intensity 16 (sz:4)
            int ind_time = offset_i + msg_lidar->fields[4].offset; // time 20 (sz:4)
            int ind_reflectivity = offset_i + msg_lidar->fields[5].offset; // reflectivity 24 (sz:2)
            int ind_ring = offset_i + msg_lidar->fields[6].offset; // ring 26 (sz:2)
            int ind_noise = offset_i + msg_lidar->fields[7].offset; // noise 28 (sz:4)
            int ind_range = offset_i + msg_lidar->fields[8].offset; // range 32 (sz: ?)

            int ind_buffer = i + ch*n_pts_channel;
            memcpy(buf_lidars_x[id]+ind_buffer,         &msg_lidar->data[ind_x],         sizeof(float));
            memcpy(buf_lidars_y[id]+ind_buffer,         &msg_lidar->data[ind_y],         sizeof(float));
            memcpy(buf_lidars_z[id]+ind_buffer,         &msg_lidar->data[ind_z],         sizeof(float));
            memcpy(buf_lidars_intensity[id]+ind_buffer, &msg_lidar->data[ind_intensity], sizeof(float));
            memcpy(buf_lidars_ring[id]+ind_buffer,      &msg_lidar->data[ind_ring],      sizeof(unsigned short));
            memcpy(buf_lidars_time[id]+ind_buffer,      &msg_lidar->data[ind_time],      sizeof(float));
        }
    }
}

void CamlidarLogger::callbackImage(const sensor_msgs::ImageConstPtr& msg, const int& id){
    cv_bridge::CvImagePtr cv_ptr;
	//cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8); // BGR8, BAYER_RGGB8 , MONO8
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // BGR8, BAYER_RGGB8 , MONO8
	*(buf_imgs_ + id) = cv_ptr->image;

    // cout << "  GCS get! [" << id << "] image.\n";
    flag_imgs_[id] = true;
};

void CamlidarLogger::callbackLidar(const sensor_msgs::PointCloud2ConstPtr& msg_lidar, const int& id){
    pointcloud2tobuffers(msg_lidar, id);
    msg_lidar->header.stamp; // timestamp

    // Create a container for the data.
    sensor_msgs::PointCloud2 output;

    // Do data processing here...
    output = *msg_lidar;

    pcl::PointCloud<pcl::PointXYZI>::Ptr temp = *(buf_lidars_ + id);
    temp->clear();
    pcl::fromROSMsg(output, *temp);

    int n_pts = temp->points.size();
    // cout <<"n_pts lidar: " <<n_pts<<endl;

    flag_lidars_[id] = true;

};


void CamlidarLogger::saveLidarData(const std::string& file_name, const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_lidar){
    int n_pts = pc_lidar->points.size();

    std::ofstream output_file(file_name, std::ios::trunc);
    output_file.precision(6);
    output_file.setf(std::ios_base::fixed, std::ios_base::floatfield);

    if(output_file.is_open()){
        output_file << "# test saving!\n";
        output_file << "# .PCD v.7 - Point Cloud Data file format\n";
        output_file << "VERSION .7\n";
        output_file << "FIELDS x y z intensity\n";
        output_file << "SIZE 4 4 4 4\n";
        output_file << "TYPE F F F F\n";
        output_file << "COUNT 1 1 1 1\n";
        output_file << "WIDTH " << n_pts << "\n";
        output_file << "HEIGHT 1\n";
        output_file << "VIEWPOINT 0 0 0 1 0 0 0\n";
        output_file << "POINTS " << n_pts<< "\n";
        output_file << "DATA ascii\n";
        for(int i = 0; i < n_pts; i++){
            output_file << pc_lidar->points[i].x<<" ";
            output_file << pc_lidar->points[i].y<<" ";
            output_file << pc_lidar->points[i].z<<" ";
            output_file << pc_lidar->points[i].intensity<<"\n";
        }
    }
};


void CamlidarLogger::saveLidarDataRingTime(const std::string& file_name, const int& id){
    int n_pts = buf_lidars_npoints[id];

    std::ofstream output_file(file_name, std::ios::trunc);
    output_file.precision(6);
    output_file.setf(std::ios_base::fixed, std::ios_base::floatfield);

    if( output_file.is_open() ) {
        output_file << "# .PCD v.7 - Point Cloud Data file format\n";
        output_file << "VERSION .7\n";
        output_file << "FIELDS x y z intensity ring time\n";
        output_file << "SIZE 4 4 4 4 2 4\n";
        output_file << "TYPE F F F F U F\n";
        output_file << "COUNT 1 1 1 1 1 1\n";
        output_file << "WIDTH " << n_pts << "\n";
        output_file << "HEIGHT 1\n";
        output_file << "VIEWPOINT 0 0 0 1 0 0 0\n";
        output_file << "POINTS " << n_pts<< "\n";
        output_file << "DATA ascii\n";
        for(int i = 0; i < n_pts; i++) {
            output_file << *(buf_lidars_x[id] + i)<<" ";
            output_file << *(buf_lidars_y[id] + i)<<" ";
            output_file << *(buf_lidars_z[id] + i)<<" ";
            output_file << *(buf_lidars_intensity[id] + i)<<" ";
            output_file << *(buf_lidars_ring[id] + i)<<" ";
            output_file << *(buf_lidars_time[id] + i)<<"\n";
        }
    }
};


void CamlidarLogger::saveAllData() {
    // save images
    bool static png_param_on = false;
	vector<int> static png_parameters;
	if (png_param_on == false) {
		png_parameters.push_back(CV_IMWRITE_PNG_COMPRESSION); // We save with no compression for faster processing
		png_parameters.push_back(0);
		png_param_on = true;
	}
    for(int id = 0; id < n_cams_;   id++) {
        string file_name = save_dir_ + "/cam"   + itos(id) + "/" + itos(current_seq_) + ".png";
	    cv::imwrite(file_name, *(buf_imgs_ + id), png_parameters);
    }

    // save lidars
    for(int id = 0; id < n_lidars_; id++) {
        string file_name = save_dir_ + "/lidar" + itos(id) + "/" + itos(current_seq_) + ".pcd";
        saveLidarDataRingTime(file_name, id);
    }

    // save association
    string file_name = save_dir_ + "/association.txt";
    std::ofstream output_file(file_name, std::ios::app);
    output_file.precision(6);
    output_file.setf(std::ios_base::fixed, std::ios_base::floatfield);
    if( output_file.is_open() ) {
        output_file << current_seq_ << " ";
        for(int i = 0; i < n_cams_; i++)   
            output_file << "/cam" << i <<   "/" << current_seq_ << ".png ";

        for(int i = 0; i < n_lidars_; i++) 
            output_file << "/lidar" << i << "/" << current_seq_ << ".pcd ";

        output_file << "\n";
    }
    printf("Save topics - seq [%d]\n", current_seq_);
    ++current_seq_;
};

#endif