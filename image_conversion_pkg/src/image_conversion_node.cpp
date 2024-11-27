#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageConversionNode : public rclcpp::Node {
public:
    ImageConversionNode() : Node("image_conversion_node") {

        auto node_ptr = shared_from_this();
        image_transport_ = std::make_shared<image_transport::ImageTransport>(node_ptr);

        
        sub_ = image_transport_->subscribe("input_image", 10, 
            std::bind(&ImageConversionNode::imageCallback, this, std::placeholders::_1));        // Subscribe to input image topic

        
        pub_ = image_transport_->advertise("output_image", 10);                                  // Advertise output image topic
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        try {
            
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);    // Convert ROS image message to OpenCV image
            
            
            cv::Mat gray_image;
            cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);                                   // Convert to grayscale

            // Convert processed OpenCV image back to ROS image message
            sensor_msgs::msg::Image::SharedPtr output_msg = 
                cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, gray_image).toImageMsg();    
            
            pub_.publish(output_msg);                // Publish the processed image

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        }
    }

    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageConversionNode>());
    rclcpp::shutdown();
    return 0;
}
