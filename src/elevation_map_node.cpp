// elevation_map_node: takes in filtered point cloud and updates existing a 2.5d map
// subscribes to: /(lidar pointcloud output) /static_tf 
// publishes to: /(topic with 2.5d map)

#include <cmath> 
#include <string>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>


namespace mapping {

class ElevationMappingNode : public rclcpp::Node 
{
    public:
    ElevationMappingNode()
    : Node("elevation_mapping_node"),
    map_({"elevation", "slope"}), // creates the layers in the 2.5D map 
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
    {
        // parameters
        map_frame_       = this->declare_parameter<std::string>("map_frame", "map");                    // frame of the map, name must exist in tf tree (check using: ros2 run tf2_tools view_frames)
        cloud_topic_     = this->declare_parameter<std::string>("cloud_topic", "/patchwork/ground_cloud"); 
        length_x_        = this->declare_parameter<double>("length_x", 30.0);                           // length in x direction of the map [m]
        length_y_        = this->declare_parameter<double>("length_y", 30.0);                           // length in y direction of the map [m]
        resolution_      = this->declare_parameter<double>("resolution", 0.3);                          // resolution of map, in length of grid size [m]
        publish_rate_    = this->declare_parameter<double>("publish_rate", 5.0);                        // publish rate in [Hz]
        reset_each_scan_ = this->declare_parameter<bool>("reset_each_scan", false);                     // to reset the map after each lidar scan
        centre_x_ = this->declare_parameter<double>("centre_x", 0.0);                                   // map centre x in map_frame [m]
        centre_y_ = this->declare_parameter<double>("centre_y", 0.0);                                   // map centre y in map_frame [m]
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");                  // rover body frame, map follows this frame
            

        // map set up
        map_.setFrameId(map_frame_);                 
        map_.setGeometry(grid_map::Length(length_x_, length_y_), resolution_, grid_map::Position(centre_x_, centre_y_));

        RCLCPP_INFO( // log output the state of the map to ensure that it is correct
            get_logger(),
            "Created map with size %f x %f m (%i x %i cells). "
            "The center of the map is located at (%f, %f) in the %s frame.",
            map_.getLength().x(), map_.getLength().y(),
            map_.getSize()(0), map_.getSize()(1),
            map_.getPosition().x(), map_.getPosition().y(), map_.getFrameId().c_str());
        clearMap();

        // subscriber
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ElevationMappingNode::cloudCb, this, std::placeholders::_1));
    
        // publisher
        pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        "elevation_map", rclcpp::QoS(1));
    
        // timer for node publishing rate
        timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_), 
        std::bind(&ElevationMappingNode::publishMap, this));
    }

    private:
        void cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            // transform point cloud from lidar into map frame
            sensor_msgs::msg::PointCloud2 cloud;  // point cloud from lidar
            try {
                auto tf = tf_buffer_.lookupTransform(
                map_frame_, msg->header.frame_id, tf2::TimePointZero);
            tf2::doTransform(*msg, cloud, tf);
            } catch (const tf2::TransformException & e) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", e.what());
                return;
            }
        
            if (reset_each_scan_) {
            clearMap();
            }

            // get rover position in map frame
            double rover_x, rover_y;
            try {
                auto base_tf = tf_buffer_.lookupTransform(
                map_frame_, base_frame_, tf2::TimePointZero);
                rover_x = base_tf.transform.translation.x;
                rover_y = base_tf.transform.translation.y;
            } catch (const tf2::TransformException & e) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", e.what());
                return;
            }

            map_.move(grid_map::Position(rover_x, rover_y));  // keep map centred on rover; move() shifts stored data, setPosition() does not

        
            // insert every point into the grid
            sensor_msgs::PointCloud2ConstIterator<float> ix(cloud, "x"), iy(cloud, "y"), iz(cloud, "z");
            for (; ix != ix.end(); ++ix, ++iy, ++iz) {
                if (!std::isfinite(*ix) || !std::isfinite(*iy) || !std::isfinite(*iz)) {
                    continue;
                }

                if (*iz < min_z_ || *iz > max_z_) continue;
            
                grid_map::Index idx; // point is outside the map
                if (!map_.getIndex(grid_map::Position(*ix, *iy), idx)) {
                    continue;  
                }
                
                // method of overwriting: just plain overwrite. can choose to take max / min / and create more layers etc
                map_.at("elevation", idx) = *iz;
            }
        }

        void clearMap()
        {
            map_["elevation"].setConstant(NAN);
        }

        void publishMap()
        {
            map_.setTimestamp(now().nanoseconds());
            auto msg = grid_map::GridMapRosConverter::toMessage(map_);
            pub_->publish(std::move(msg));
        }


        // parameters
        std::string map_frame_, cloud_topic_, base_frame_;
        double length_x_{30.0}, length_y_{30.0}, resolution_{0.1}, publish_rate_{5.0};
        double centre_x_{0.0}, centre_y_{0.0};
        double min_z_{-3.0}, max_z_{6.0};  // metres relative to the rover's height
        bool reset_each_scan_{false};
        
        // State (declaration order matters: tf_buffer_ must come before tf_listener_)
        grid_map::GridMap map_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;

    };
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mapping::ElevationMappingNode>());
  rclcpp::shutdown();
  return 0;
}