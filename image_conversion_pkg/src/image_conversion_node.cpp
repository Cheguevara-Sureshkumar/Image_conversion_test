#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageConversionNode : public rclcpp::Node {
public:
    ImageConversionNode() : Node("image_conversion_node") {
        // Create a shared pointer to the current node for ImageTransport
        auto node_ptr = shared_from_this();
        image_transport_ = std::make_shared<image_transport::ImageTransport>(node_ptr);

        // Subscribe to input image topic
        sub_ = image_transport_->subscribe("input_image", 10, 
            std::bind(&ImageConversionNode::imageCallback, this, std::placeholders::_1));

        // Advertise output image topic
        pub_ = image_transport_->advertise("output_image", 10);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        try {
            // Convert ROS image message to OpenCV image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            
            // Perform image processing (example: convert to grayscale)
            cv::Mat gray_image;
            cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

            // Convert processed OpenCV image back to ROS image message
            sensor_msgs::msg::Image::SharedPtr output_msg = 
                cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, gray_image).toImageMsg();

            // Publish the processed image
            pub_.publish(output_msg);

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
