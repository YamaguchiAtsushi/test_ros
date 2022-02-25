#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

class ImagePublisher{
public:
    ros::NodeHandle nh_;
    //ros::Publisher imagePub_;
    std::string imagesDir_;
    image_transport::ImageTransport it_;
    image_transport::Publisher imagePub_;

    ImagePublisher(void):
        nh_("~"),
        imagesDir_("/home/yamaguchi/test_ros/catkin_ws/images/"),
        it_(nh_)
    {
        //imagePub_ = nh_.advertise<sensor_msgs::Image>("/image_test",1);
        imagePub_ = it_.advertise("/image_test", 1);
    }

    void spin(void){
        ros::Rate loopRate(1);
        int imageIdx = 0;
        while(ros::ok()){
            ros::spinOnce();

            std::string imageFileName = imagesDir_ + std::to_string(imageIdx) + ".jpg";
            //printf("imageFileName = %s\n", imageFileName.c_str());
            cv::Mat image = cv::imread(imageFileName, 1);
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            header.frame_id = "/image";
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
            imagePub_.publish(msg);
            
            imageIdx++;
            if(imageIdx > 4){
                imageIdx = 0;
            }
            loopRate.sleep();
        }
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "image_publisher");
    ImagePublisher publisher;
    publisher.spin();
    return 0;
}