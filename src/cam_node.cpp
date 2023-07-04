#include "camera/cam_node.hpp"


Cam_Node::Cam_Node(ros::NodeHandle *nh, int a) 
{
    std::string ns = ros::this_node::getNamespace();
    std::string s = std::to_string(a);
    cam_data_pub_Twist = nh->advertise<geometry_msgs::Twist>(ns + "/april_data_Twist"+s, 1);
    
    //cam_data_pub_rot = nh->advertise<geometry_msgs::Point>(ns + "/april_data_rot"+s, 1);
    //cam_data_pub_pos = nh->advertise<geometry_msgs::Point>(ns + "/april_data_pos"+s, 1);
    // cam_data_pub = nh->advertise<geometry_msgs::Point>(ns + "/april_data", 1);
    img_raw_sub = nh->subscribe("/camera/color/image_raw", 1, &Cam_Node::img_raw_callback, this);
    
};

void Cam_Node::img_raw_callback(const sensor_msgs::Image::ConstPtr& msg)
{
    const float markerLength = 0.12;
    cv::Mat imageCopy;
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    cv_ptr->image.copyTo(imageCopy);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;

    int flag = 0;

    auto ms = geometry_msgs::Twist();

    // Creating 2nd publisher for rotation
    //auto ms_pos = geometry_msgs::Point();

    // Detect this type of aruco
    cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids);
    int size = ids.size();
    
    if (size > 0)
    {    
        // chooses a specific id to detect

        for (int j = size; j > 0; j-- )
        {
            if (ids[j-1] == id_req)
            {
                i = j-1;
                flag = 1;
                break;
            }
        }
        cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

        // Create a vector to store rvecs and tvecs
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
        
        cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[0], tvecs[0], 0.2);
        cv::Vec3d tvec = tvecs[i];
        cv::Vec3d rvec = rvecs[i];
        float id = ids[i];


        // Does this make sense? rvec should be a vector part of the rotation matrix
        float rx = rvec[0];
        float ry = rvec[1];
        float rz = rvec[2];


        float x = tvec[0];  //left and right
        float y = tvec[1];  //depth into camera .... offset is relative to coordinate frame of april tag
        float z = tvec[2];  // .... offset is relative to coordinate frame of april tag

        ms.linear.x = x;
        ms.linear.y = y;
        ms.linear.z = z;

        ms.angular.x = rx;
        ms.angular.y = ry;
        ms.angular.z = rz;

        // i = i + 1;

        if (flag == 1)
        {
            cam_data_pub_Twist.publish(ms);
            //cam_data_pub_pos.publish(ms_pos);
        }


        // std::cout << "Z direction: "<< z << "\n"; 
        // float dist = sqrt(pow(x,2) + pow(y,2) + pow(z,2));
        // float angle = std::atan2(y, x);


    }
    cv::imshow(OPENCV_WINDOW, imageCopy);
    cv::waitKey(1);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh;
    const int agent_idx = ros::this_node::getName().substr(ros::this_node::getNamespace().size()).back() - '0';
    Cam_Node camnode = Cam_Node(&nh, agent_idx);
    camnode.i = 0;

    
    //if (argc == 2)
    //{
    //    camnode.id = std::stoi(argv[1]);
    //}
    camnode.id_req = agent_idx;

    while(ros::ok())
    {
        ros::spinOnce();

        //if (camnode.i > 0){
        //    ros::shutdown();
        //}
    }
    
    return 0;
}
