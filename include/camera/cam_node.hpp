#ifndef CAM_NODE_h
#define CAM_NODE_h

#include <chrono>
#include <memory>
#include <functional>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/aruco.hpp"
#include "opencv2/opencv.hpp"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"


class Cam_Node {
    public:
        Cam_Node(ros::NodeHandle *nh, int a);

        //April Tag length
        double markerLength;
        int i = 0;
        int id_req = 0;
    
    private:

        const float Zoffset = 0.06;               // relative to coordinate frame of april tag

        const float Yoffset = 0.053;               // relative to coordinate frame of april tag

        // get raw image data
        void img_raw_callback(const sensor_msgs::Image::ConstPtr& msg);
        
        cv_bridge::CvImagePtr cv_ptr;

        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);


        const std::string OPENCV_WINDOW = "Image window";
        
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) 
                            << 1368.7261962890625, 0.0,           971.5208740234375, 
                                0.0,            1368.89013671875, 553.1422119140625, 
                                0.0,              0.0,              1.0);

        //cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) 
        //                    << 610.3250122070312, 0.0,             331.69219970703125, 
        //                        0.0,            610.5338134765625, 239.29869079589844, 
        //                        0.0,              0.0,              1.0);

        cv::Mat distCoeffs = (cv::Mat_<double>(1,5) << 0.0, 0.0, 0.0, 0.0, 0.0);

        //ROS Node stuff
        //ros::Publisher cam_data_pub_rot;
        //ros::Publisher cam_data_pub_pos;

        ros::Publisher cam_data_pub_Twist;
        ros::Subscriber img_raw_sub;
};





#endif
