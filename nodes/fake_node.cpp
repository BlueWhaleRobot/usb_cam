#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sensor_msgs/fill_image.h>

namespace usb_cam
{

class FakeNode
{
public:
    ros::NodeHandle node_;
    sensor_msgs::Image img_;
    image_transport::CameraPublisher image_pub_;

    std::string video_device_name_, io_method_name_, pixel_format_name_, camera_name_, camera_info_url_;
    //std::string start_service_name_, start_service_name_;
    bool streaming_status_;
    int image_width_, image_height_, framerate_, exposure_, brightness_, contrast_, saturation_, sharpness_, focus_,
        white_balance_, gain_, time_delay_;
    bool autofocus_, autoexposure_, auto_white_balance_;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

    ros::ServiceServer service_start_, service_stop_;

    FakeNode() : node_("~")
    {
        image_transport::ImageTransport it(node_);
        image_pub_ = it.advertiseCamera("image_raw", 1);

        // grab the parameters
        node_.param("video_device", video_device_name_, std::string("/dev/video0"));
        node_.param("brightness", brightness_, -1); //0-255, -1 "leave alone"
        node_.param("contrast", contrast_, -1);     //0-255, -1 "leave alone"
        node_.param("saturation", saturation_, -1); //0-255, -1 "leave alone"
        node_.param("sharpness", sharpness_, -1);   //0-255, -1 "leave alone"
        // possible values: mmap, read, userptr
        node_.param("io_method", io_method_name_, std::string("mmap"));
        node_.param("image_width", image_width_, 640);
        node_.param("image_height", image_height_, 480);
        node_.param("framerate", framerate_, 30);
        node_.param("time_delay", time_delay_, 30);
        // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
        node_.param("pixel_format", pixel_format_name_, std::string("mjpeg"));
        // enable/disable autofocus
        node_.param("autofocus", autofocus_, false);
        node_.param("focus", focus_, -1); //0-255, -1 "leave alone"
        // enable/disable autoexposure
        node_.param("autoexposure", autoexposure_, true);
        node_.param("exposure", exposure_, 100);
        node_.param("gain", gain_, -1); //0-100?, -1 "leave alone"
        // enable/disable auto white balance temperature
        node_.param("auto_white_balance", auto_white_balance_, true);
        node_.param("white_balance", white_balance_, 4000);

        // load the camera info
        node_.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
        node_.param("camera_name", camera_name_, std::string("head_camera"));
        node_.param("camera_info_url", camera_info_url_, std::string(""));
        cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_, camera_info_url_));

        if (!cinfo_->isCalibrated())
        {
            cinfo_->setCameraName(video_device_name_);
            sensor_msgs::CameraInfo camera_info;
            camera_info.header.frame_id = img_.header.frame_id;
            camera_info.width = image_width_;
            camera_info.height = image_height_;
            cinfo_->setCameraInfo(camera_info);
        }
    }

    bool take_and_send_image()
    {
        // 绘制图像
        cv::Mat cv_image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::stringstream ss;
        ss << "Current time: " << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
        int baseline = 0;
        cv::Size textSize = cv::getTextSize(ss.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
        cv_image.rowRange(cv_image.rows - textSize.height - 10, cv_image.rows) = cv::Mat::zeros(textSize.height + 10, cv_image.cols, cv_image.type());
        cv::putText(cv_image, ss.str(), cv::Point(5, cv_image.rows - 5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);
        // 转成图像消息
        cv_bridge::CvImage bridge;
        cv_image.copyTo(bridge.image);
        bridge.encoding = "bgr8";
        bridge.header.frame_id = img_.header.frame_id;
        bridge.header.stamp = ros::Time::now();
        img_ = *(bridge.toImageMsg());
        // grab the camera info
        sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
        ci->header.frame_id = img_.header.frame_id;
        ci->header.stamp = img_.header.stamp;
        // publish the image
        image_pub_.publish(img_, *ci);
        return true;
    }

    bool spin()
    {
        ros::Rate loop_rate(this->framerate_);
        while (node_.ok())
        {
            if (!take_and_send_image()) ROS_WARN("USB camera did not respond in time.");
            ros::spinOnce();
            loop_rate.sleep();
        }
        return true;
    }
};
} // namespace usb_cam

int main(int argc, char **argv)
{
    ros::init(argc, argv, "usb_cam");
    usb_cam::FakeNode a;
    a.spin();
    return EXIT_SUCCESS;
}
