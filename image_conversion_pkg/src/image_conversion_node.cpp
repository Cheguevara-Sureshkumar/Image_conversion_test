#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_srvs/srv/set_bool.hpp>

class ImageConversionNode : public rclcpp::Node {
public:
    ImageConversionNode() : Node("image_conversion_node"), grayscale_mode_(true) {
        this->declare_parameter<std::string>("input_topic", "/image_raw");
        this->declare_parameter<std::string>("output_topic", "/converted_image");

        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();

        image_transport_ = std::make_shared<image_transport::ImageTransport>(
            std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}));

        sub_ = image_transport_->subscribe(input_topic, 10, 
            std::bind(&ImageConversionNode::imageCallback, this, std::placeholders::_1));    // Subscriber of the input image

        pub_ = image_transport_->advertise(output_topic, 10);                                // Publisher of the output image
        
        service_ = this->create_service<std_srvs::srv::SetBool>("change_mode", 
            std::bind(&ImageConversionNode::changeModeCallback, this, 
            std::placeholders::_1, std::placeholders::_2)
        );                                                                                   // Creation of service for conversion
    }
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        try {
	    if (msg->encoding != sensor_msgs::image_encodings::BGR8) {
	            RCLCPP_WARN(this->get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str());
	            return;
	    }
	    else{
	            // Convert ROS Image message to OpenCV format
	            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	            
	            cv::Mat processed_image;
	            if (grayscale_mode_) {
	                
	                cv::cvtColor(cv_ptr->image, processed_image, cv::COLOR_BGR2GRAY);    // Convert the image to grayscale
	                
	                sensor_msgs::msg::Image::SharedPtr output_msg = 
	                    cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, processed_image).toImageMsg();    // Convert OpenCV image back to ROS Image message
	                
	                pub_.publish(output_msg);        // Publish the converted grayscale image
	            } 
	            else {
	                
	                sensor_msgs::msg::Image::SharedPtr output_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, cv_ptr->image).toImageMsg();  // Convert OpenCV image back to ROS Image message
			        pub_.publish(output_msg);    // Publish the original color image
	            }
	    }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        }
    }
    void changeModeCallback(
        const std_srvs::srv::SetBool::Request::SharedPtr request,
        const std_srvs::srv::SetBool::Response::SharedPtr response) {
        
        grayscale_mode_ = request->data;
        
        RCLCPP_INFO(this->get_logger(), "Image conversion mode changed to: %s", 
                    grayscale_mode_ ? "Grayscale" : "Color");        
        response->success = true;
        response->message = grayscale_mode_ ? 
            "Switched to Grayscale mode" : "Switched to Color mode";
    }
private:
    std::shared_ptr<image_transport::ImageTransport> image_transport_;  

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;
    bool grayscale_mode_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageConversionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
