#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <FP_Magang/PC2BS.h>
#include <FP_Magang/PC2PC.h>

ros::Publisher pub;
ros::Publisher pub2;

void publishTargetPosition(float target_x, float target_y) {
    // FP_Magang::PC2BS control_msg;
    // control_msg.bola_x = target_x;
    // control_msg.bola_y = target_y;
    if (target_x == 0 || target_y == 0) {
        return;
    }
    // // pub.publish(control_msg);

    std::cout << target_x << " " << target_y << std::endl;

    FP_Magang::PC2PC control_msg_2;
    control_msg_2.follow_x = target_x;
    control_msg_2.follow_y = target_y;
    pub2.publish(control_msg_2);
}

void processImage(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv::Mat img(msg->height, msg->width, CV_8UC3, const_cast<uint8_t*>(&msg->data[0]), msg->step);
        cv::Mat hsv_image;
        cv::cvtColor(img, hsv_image, cv::COLOR_BGR2HSV);
        cv::Scalar lower_orange(5, 150, 150);
        cv::Scalar upper_orange(15, 255, 255);
        cv::Mat mask;
        cv::inRange(hsv_image, lower_orange, upper_orange, mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            int largest_contour_index = 0;
            double max_area = 0;
            for (int i = 0; i < contours.size(); i++) {
                double area = cv::contourArea(contours[i]);
                if (area > max_area) {
                    max_area = area;
                    largest_contour_index = i;
                }
            }

            cv::Moments moments = cv::moments(contours[largest_contour_index]);
            int center_x = int(moments.m10 / moments.m00);
            int center_y = int(moments.m01 / moments.m00);

            publishTargetPosition(center_x, center_y);
        }
    }
    catch (cv::Exception& e) {
        ROS_ERROR("Error processing image: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_processor");

    ros::NodeHandle nh;

    ros::Subscriber image_sub = nh.subscribe("/image_resized", 2, processImage);
    pub = nh.advertise<FP_Magang::PC2BS>("/pc2bs", 1);
    pub2 = nh.advertise<FP_Magang::PC2PC>("/pc2pc", 1);

    ros::spin();
    return 0;
}
