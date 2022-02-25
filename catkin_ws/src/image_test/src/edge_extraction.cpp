#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

class EdgeExtraction{
public:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber imageSub_;
    image_transport::Publisher imagePub_;

    EdgeExtraction(void):
        nh_("~"),
        it_(nh_)
    {
        imageSub_ = it_.subscribe("/image_test", 1, &EdgeExtraction::imageCB, this);
        imagePub_ = it_.advertise("/edge_image_test", 1);
    }

    void spin(void){
        ros::spin();
    }

    void imageCB(const sensor_msgs::Image::ConstPtr &msg){
        cv_bridge::CvImagePtr cvPtr;
        try{
            cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }catch(cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat edge;
        cv::Canny(cvPtr->image, edge, 100.0, 200.0);
        std_msgs::Header header;
        header = msg->header;
        sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(header, "mono8", edge).toImageMsg();
        imagePub_.publish(imageMsg);
        
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "edge_extraction");
    EdgeExtraction extractor;
    extractor.spin();
    return 0;
}