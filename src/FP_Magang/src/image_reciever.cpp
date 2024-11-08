#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>

ros::Publisher image_pub;

void processAndPublishImage(cv::Mat& frame) {
    // Resize image to 900x600
    cv::resize(frame, frame, cv::Size(900, 600));

    // Convert image to ROS message format
    sensor_msgs::Image img_msg;
    img_msg.header.stamp = ros::Time::now();
    img_msg.header.frame_id = "camera";
    img_msg.height = frame.rows;
    img_msg.width = frame.cols;
    img_msg.encoding = "bgr8";  // We use BGR8 encoding for OpenCV
    img_msg.is_bigendian = false;
    img_msg.step = frame.cols * 3;  // 3 bytes per pixel (BGR)
    img_msg.data.resize(img_msg.step * img_msg.height);

    // Copy image data into ROS message
    memcpy(&img_msg.data[0], frame.data, frame.total() * frame.elemSize());

    // Publish the resized image
    image_pub.publish(img_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_receiver");
    ros::NodeHandle nh;

    // Initialize the publisher
    image_pub = nh.advertise<sensor_msgs::Image>("/image_resized", 1);

    // Load the image file (static image)
    cv::Mat frame = cv::imread("/home/rasya/Documents/ROBOTIK/FINAL_PROJECT/fp_ws/src/bola1.jpg");

    if (frame.empty()) {
        ROS_ERROR("Failed to open image.");
        return 1;
    }

    ros::Rate loop_rate(1);  // 30 frames per second

    // Simulate publishing the image in a loop
    while (ros::ok()) {
        // Since it's a static image, we simply call the processAndPublishImage function.
        processAndPublishImage(frame);

        // Allow ROS to process any pending callbacks and ensure continuous publishing
        ros::spinOnce();

        // Sleep to maintain the desired loop rate (30 FPS)
        loop_rate.sleep();
    }

    return 0;
}
