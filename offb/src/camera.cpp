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


    std::cout << "before cv::VideoCapture capture without gst-launch-1.0" << std::endl;
    // cv::VideoCapture capture("udpsrc port=5600 ! application/x-rtp media=video payload=26 clock-rate=90000 encoding-name=JPEG framerate=15/1 ! rtpjpegdepay ! jpegdec ! videoconvert ! appsink", cv::CAP_GSTREAMER);
    // cv::VideoCapture capture("uvch264src initial-bitrate=1000000 average-bitrate=1000000 iframe-period=1000 device=/dev/video0 name=src auto-start=true src.vidsrc ! video/x-h264,width=640,height=360,framerate=15/1 ! h264parse ! rtph264pay ! udpsink host=192.168.2.1 port=5600", cv::CAP_GSTREAMER);
    cv::VideoCapture capture("udpsrc address=192.168.2.1 port=5600 auto-multicast=0 ! application/x-rtp,media=video,encoding-name=H264 ! tpjitterbuffer latency=0 ! rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1", cv::CAP_GSTREAMER);
    std::cout << "cv::VideoCapture capture constructed" << std::endl;


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



