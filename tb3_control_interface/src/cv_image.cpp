#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>





int main(int argc, char** argv)
{
 ros::init(argc, argv, "image_converter");
 cv::Mat img=cv::imread("/home/davorb/map_cropped.png",cv::IMREAD_GRAYSCALE);
 //cv::imshow("Image window", img);
 cv::Vec3b piksel=img.at<cv::Vec3b>(0,0);
 img.at<uchar>(50,3)=100;
 
int up_width = 600;
int up_height = 600;
cv::Mat resized_up;
  //resize up
cv::resize(img, resized_up, cv::Size(up_width, up_height), cv::INTER_LINEAR);
 
 
 
 cv::imshow("slika",resized_up);
 cv::waitKey();
 //cv::waitKey();
 ros::spin();
 return 0;
}
