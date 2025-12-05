#ifndef WAREHOUSE_ROVER_DATABASE__INVENTORY_NODE_HPP_
#define WAREHOUSE_ROVER_DATABASE__INVENTORY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "warehouse_rover_msgs/msg/qr_detection_array.hpp"
#include "warehouse_rover_database/database_manager.hpp"
#include <memory>

namespace warehouse_rover_database
{

class InventoryNode : public rclcpp::Node
{
public:
  explicit InventoryNode(const rclcpp::NodeOptions & options);
  ~InventoryNode();

private:
  void qrCallback(const warehouse_rover_msgs::msg::QRDetectionArray::SharedPtr msg);
  void startNewMission();
  
  rclcpp::Subscription<warehouse_rover_msgs::msg::QRDetectionArray>::SharedPtr qr_sub_;
  
  std::unique_ptr<DatabaseManager> db_;
  
  std::string current_mission_id_;
  std::string db_path_;
  bool auto_export_;
  std::string export_dir_;
};

}  // namespace warehouse_rover_database

#endif
