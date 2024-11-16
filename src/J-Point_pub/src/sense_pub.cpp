#include "rclcpp/rclcpp.hpp"
#include <librealsense2/rs.hpp> 
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace rs2;

class J_Point_Publisher : public rclcpp::Node{
    public:
        J_Point_Publisher(): Node("realsense_pointcloud_publisher"){
            pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("realsense/pointcloud", 10);
            pipe_.start();
        }
    private:
    void publish_pointcloud(){
        // Wait for a new frame from the RealSense camera
        rs2::frameset frameset = pipe_.wait_for_frames();
        
        // Get the depth frame
        rs2::depth_frame depth = frameset.get_depth_frame();

        // Create PointCloud2 message
        sensor_msgs::msg::PointCloud2 pointcloud_msg;
        pointcloud_msg.header.stamp = this->get_clock()->now();
        pointcloud_msg.header.frame_id = "camera_link"; // Adjust frame_id based on your setup
        
        // Set PointCloud2 fields: x, y, z, and intensity (optional)
        pointcloud_msg.fields.resize(4);
        pointcloud_msg.fields[0].name = "x";
        pointcloud_msg.fields[0].offset = 0;
        pointcloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloud_msg.fields[0].count = 1;

        pointcloud_msg.fields[1].name = "y";
        pointcloud_msg.fields[1].offset = 4;
        pointcloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloud_msg.fields[1].count = 1;

        pointcloud_msg.fields[2].name = "z";
        pointcloud_msg.fields[2].offset = 8;
        pointcloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloud_msg.fields[2].count = 1;

        pointcloud_msg.fields[3].name = "intensity";
        pointcloud_msg.fields[3].offset = 12;
        pointcloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloud_msg.fields[3].count = 1;

        // Allocate data for the point cloud
        pointcloud_msg.data.resize(pointcloud_msg.row_step * pointcloud_msg.height);
        uint8_t *ptr = pointcloud_msg.data.data();

        for (int y = 0; y < depth.get_height(); ++y)
        {
            for (int x = 0; x < depth.get_width(); ++x)
            {
                // Get depth value
                float depth_value = depth.get_distance(x, y);

                // Set x, y, z based on depth
                float x_val = (x - depth.get_width() / 2) * depth_value;
                float y_val = (y - depth.get_height() / 2) * depth_value;
                float z_val = depth_value;

                // Store the values in the message
                memcpy(ptr, &x_val, sizeof(float)); ptr += sizeof(float);
                memcpy(ptr, &y_val, sizeof(float)); ptr += sizeof(float);
                memcpy(ptr, &z_val, sizeof(float)); ptr += sizeof(float);
                
                // Optionally, set intensity (can be depth or any other data)
                float intensity = depth_value;
                memcpy(ptr, &intensity, sizeof(float)); ptr += sizeof(float);
            }
        }
        // Publish the PointCloud2 message
        pointcloud_pub_->publish(pointcloud_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rs2::pipeline pipe_;

};


int main(){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealSensePublisher>());
    rclcpp::shutdown();
    return 0;

    // window app(1280, 720, "J-Point_Pub");

    // glfw_state app_state;

    // register_glfw_callbacks(app, app_state);

    // rs2::pointcloud pc;

    // rs2::points points;

    // pipeline pipe;

    // pipe.start();

    // while(app){

    //     auto frames = pipe.wait_for_frames();
    //     auto depth = frames.get_depth_frame();

    //     points = pc.calculate(depth);

    //     auto color = frames.get_color_frame();

    //     pc.map_to(color);

    //     app_state.tex.upload(color);

    // }

    // return EXIT_SUCCESS;

}
