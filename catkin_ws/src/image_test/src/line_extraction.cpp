#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

class LineExtraction{
public:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber imageSub_;
    image_transport::Publisher imagePub_;

    LineExtraction(void):
        nh_("~"),
        it_(nh_)
    {
        imageSub_ = it_.subscribe("/edge_image_test", 1, &LineExtraction::imageCB, this);
        imagePub_ = it_.advertise("/line_image_test", 1);
    }

    void spin(void){
        ros::spin();
    }

    void imageCB(const sensor_msgs::Image::ConstPtr &msg){
        cv_bridge::CvImagePtr cvPtr;
        try{
            cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }catch(cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
    }

    std::vector<cv::Vec2f> lines;
    cv::HoughLines(cvPtr->image, lines, 1.0, CV_PI / 180.0, 250, 0, 0);
    std::vector<cv::Vec2f>::iterator it = lines.begin();
    cv::Mat lineImage = cv::Mat::zeros(cvPtr->image.rows, cvPtr->image.cols, CV_8UC3 );
    for(; it != lines.end(); ++it){
        float rho = (*it)[0], theta = (*it)[1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a * rho, y0 = b * rho;
        pt1.x = cv::saturate_cast<int>(x0 + 1000 * (-b));
        pt1.y = cv::saturate_cast<int>(y0 + 1000 * (a));
        pt2.x = cv::saturate_cast<int>(x0 - 1000 * (-b));
        pt2.y = cv::saturate_cast<int>(y0 - 1000 * (a));
        cv::line(lineImage, pt1, pt2, cv::Scalar(0, 0, 255), 3, 8);

    }


        std_msgs::Header header;
        header = msg->header;
        sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(header, "bgr8", lineImage).toImageMsg();
        imagePub_.publish(imageMsg);
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "line_extraction");
    LineExtraction extractor;
    extractor.spin();
    return 0;
}