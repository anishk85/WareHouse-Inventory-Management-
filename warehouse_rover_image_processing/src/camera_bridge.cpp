#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <cstdio>
#include <array>
#include <vector>
#include <opencv2/opencv.hpp>
#include <unistd.h>

static const int WIDTH = 640;
static const int HEIGHT = 480;

class CameraBridge : public rclcpp::Node
{
public:
    CameraBridge() : Node("camera_bridge")
    {
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 30);

        // FFmpeg command
        std::string cmd =
            "/bin/bash -lc \"ffmpeg "
            "-loglevel debug "
            "-i tcp://0.0.0.0:8000?listen "
            "-f rawvideo -pix_fmt bgr24 -\"";


        fp_ = popen(cmd.c_str(), "r");
        if (!fp_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start FFmpeg!");
            throw std::runtime_error("FFmpeg start failed");
        }


        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&CameraBridge::read_frame, this));

        RCLCPP_INFO(this->get_logger(), "Camera bridge waiting for video stream...");
    }

    ~CameraBridge()
    {
        if (fp_) pclose(fp_);
    }

private:
    void read_frame()
    {
        size_t frame_size = WIDTH * HEIGHT * 3;
        std::vector<unsigned char> buffer(frame_size);

        size_t read_bytes = fread(buffer.data(), 1, frame_size, fp_);

        if (read_bytes != frame_size) {
            return; // No frame yet
        }

        cv::Mat img(HEIGHT, WIDTH, CV_8UC3, buffer.data());

        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera_frame";

        sensor_msgs::msg::Image msg;
        cv_bridge::CvImage(header, "bgr8", img).toImageMsg(msg);

        pub_->publish(msg);
    }

    FILE* fp_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraBridge>());
    rclcpp::shutdown();
    return 0;
}
