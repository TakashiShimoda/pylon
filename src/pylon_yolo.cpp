#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sys/time.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <stdio.h>

class ImageInput{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    private:
       cv_bridge::CvImagePtr bgr_image;

    public:
       ImageInput()
           :it_(nh_)
       {
           image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageInput::image_callback, this);
       }
        void image_callback(const sensor_msgs::ImageConstPtr& msg){
            try{
                bgr_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
        }
        //画像へのアクセサ
        cv::Mat getImg(){
            return bgr_image->image; 
        }
};

class PylonDetector{
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    ros::NodeHandle nh_;
    ImageInput image ;
    image = new ImageInput();
    public:
        PylonDetector()
           :it_(nh_)
        {
            image_pub_ = it_.advertise("camera_image", 1);
            main_loop();
        }
        void outline_detection();
        void pub_img(cv::Mat frame);
        void main_loop();
};

void PylonDetector::main_loop(){
    ros::Rate loop_rate(30);
    while (ros::ok()){
        outline_detection();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void PylonDetector::outline_detection(){
    cv::Mat src = image->getImg();
    cv::Mat gray;
    cv::cvtColor(src, gray, CV_BGR2GRAY);
    pub_img(gray);
}

void PylonDetector::pub_img(cv::Mat frame){
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    image_pub_.publish(msg);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "pylon_detector");
    PylonDetector pylon;
    pylon = new PylonDetector();
    return 0;
}