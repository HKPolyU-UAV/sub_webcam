#include <sstream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include "geometry_msgs/PointStamped.h"


// #include "include/run_yolo.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include "offb/obj.h"

using namespace std;


static int counter = 0;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub_webcam");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/ardusub/image", 1);

    cv::VideoCapture capture(0);
 
	if (!capture.isOpened())
	{
		std::cout << "Read video Failed !" << std::endl;
		return 0;
	}
 
	cv::Mat frame;
	// cv::namedWindow("video test");
 
	int frame_num = capture.get(cv::CAP_PROP_FRAME_COUNT);
	std::cout << "total frame number is: " << frame_num << std::endl;

    while (ros::ok())
    {
        capture >> frame;
        // cv::imshow("hi", frame);
        // cv::waitKey(20);
        cout<<frame.size()<<endl;

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);
        ros::spinOnce();
    }
    
 
	// cv::destroyWindow("video test");
	capture.release();

    return 0;
}



