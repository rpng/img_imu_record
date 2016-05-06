#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/image_encodings.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include "boost/filesystem.hpp"
#include "boost/bind.hpp"


using namespace std;

// Global variables (hehe, these should not be here)
std::string folder_root;
std::string folder_images;
std::string folder_images_l;
std::string folder_images_r;
std::string folder_data;
std::string data_imus;
std::string data_imgs;
std::ofstream outfile_imu;
std::ofstream outfile_img;

// Function predeclare headers (hehe, yet again, should not be here)
void imu_callback(sensor_msgs::Imu msg);
void image_mono_callback(const sensor_msgs::ImageConstPtr &msg);
void image_stereo_callback(const sensor_msgs::ImageConstPtr& msgL, const sensor_msgs::ImageConstPtr& msgR);
void create_directory(std::string path);


int main(int argc, char **argv) {

    // Startup the ros node
    ros::init(argc, argv, "img_imu_record");
    ros::NodeHandle nh;

    // Get config params
    bool is_stereo = true;

    // Subscribe to our img and imu topics
    ros::Subscriber sub_imu = nh.subscribe("/imu0", 1000, imu_callback);
    //ros::Subscriber sub_img = nh.subscribe("/camera/image_raw", 1000, image_mono_callback);


    // Subscribe to the right topics based on stereo configuration
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/cam0/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/cam1/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    sync.registerCallback(boost::bind(&image_stereo_callback,_1,_2));

    // Create our folder timestamp
    char filename[128];
    std::sprintf(filename, "%ld", ros::Time::now().toNSec());

    // Create folder and file paths
    folder_root = ros::package::getPath("img_imu_record")+"/recordings/"+filename+"/";

    folder_data = ros::package::getPath("img_imu_record")+"/recordings/"+filename+"/data/";
    data_imus = ros::package::getPath("img_imu_record")+"/recordings/"+filename+"/data/imu_data.txt";
    data_imgs = ros::package::getPath("img_imu_record")+"/recordings/"+filename+"/data/img_data.txt";

    // Create the folders
    create_directory(folder_root);
    create_directory(folder_data);

    // If not stereo
    if(!is_stereo) {
        folder_images = ros::package::getPath("img_imu_record")+"/recordings/"+filename+"/images/";
        create_directory(folder_images);
    }
    // If we are in stereo, create the sub folders
    else {
        folder_images_l = ros::package::getPath("img_imu_record")+"/recordings/"+filename+"/images_cam0/";
        folder_images_r = ros::package::getPath("img_imu_record")+"/recordings/"+filename+"/images_cam1/";
        create_directory(folder_images_l);
        create_directory(folder_images_r);
    }

    // Open our imu file and keep it open
    outfile_imu.open(data_imus.c_str(), std::ios_base::app);
    outfile_img.open(data_imgs.c_str(), std::ios_base::app);

    ROS_INFO("Done loading information, writing information out");

    ros::spin();

    outfile_imu.close();
    outfile_img.close();

    return 0;
}

/**
 * This method will be called when a new imu message is published
 * We will write this information to the open imu file
 */
void imu_callback(sensor_msgs::Imu msg) {
    if (!outfile_imu.is_open()) {
        ROS_ERROR("Unable to open imu file");
        return;
    }
    // Else write the new reading to file
    outfile_imu << msg.angular_velocity.x << " "
                << msg.angular_velocity.y << " "
                << msg.angular_velocity.z << " "
                << msg.linear_acceleration.x << " "
                << msg.linear_acceleration.y << " "
                << msg.linear_acceleration.z << " "
                << msg.header.stamp.toNSec() << "\n";
}

/**
 * This method will be called when a new image message is published
 * We will write this information to the image folder
 * We will append the image time to the timestamp of the folder
 */
void image_mono_callback(const sensor_msgs::ImageConstPtr &msg) {
    if (!outfile_img.is_open()) {
        ROS_ERROR("Unable to open image file");
        return;
    }

    // Create our folder timestamp
    char filename[128];
    std::sprintf(filename, "img_%ld.png", msg.get()->header.stamp.toNSec());

    // Else write the new reading to file
    outfile_img << msg.get()->height << " "
                << msg.get()->width << " "
                << msg.get()->header.stamp.toNSec() << " "
                << "images/" << filename << "\n";

    // Convert the image msg to open cv
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::imwrite(folder_images+filename, cv_ptr->image);
}

/**
 * This method will be called when a new image message is published
 * We will write this information to the image folder
 * We will append the image time to the timestamp of the folder
 */
void image_stereo_callback(const sensor_msgs::ImageConstPtr& msgL,const sensor_msgs::ImageConstPtr& msgR) {
    if (!outfile_img.is_open()) {
        ROS_ERROR("Unable to open image file");
        return;
    }

    // Create our folder timestamp
    char filename[128];
    std::sprintf(filename, "img_%ld.png", msgL.get()->header.stamp.toNSec());

    // Else write the new reading to file
    outfile_img << msgL.get()->height << " "
                << msgL.get()->width << " "
                << msgL.get()->header.stamp.toNSec() << " "
                << "images_cam0/" << filename << " "
                << msgR.get()->height << " "
                << msgR.get()->width << " "
                << msgR.get()->header.stamp.toNSec() << " "
                << "images_cam1/" << filename << "\n";

    // Convert the image msg to open cv
    cv_bridge::CvImagePtr cv_ptr_l = cv_bridge::toCvCopy(msgL, "bgr8");
    cv::imwrite(folder_images_l+filename, cv_ptr_l->image);
    cv_bridge::CvImagePtr cv_ptr_r = cv_bridge::toCvCopy(msgR, "bgr8");
    cv::imwrite(folder_images_r+filename, cv_ptr_r->image);
}


/**
 * Helper function, this will create the path passed to it
 * It will return if the directory is already created
 */
void create_directory(std::string path) {
    // Boost object
    boost::filesystem::path dir(path.c_str());
    // Check to see if directory is already created
    if(boost::filesystem::exists(dir))
        return;
    // Create the directory
    boost::filesystem::create_directories(dir);
}