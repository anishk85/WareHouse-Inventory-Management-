#include "warehouse_rover_database/inventory_node.hpp"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>

namespace fs = std::filesystem;

namespace warehouse_rover_database
{

InventoryNode::InventoryNode(const rclcpp::NodeOptions & options)
: Node("inventory_node", options)
{
  // Parameters
  this->declare_parameter<std::string>("database_path", "/tmp/warehouse_inventory.db");
  this->declare_parameter<bool>("auto_export", true);
  this->declare_parameter<std::string>("export_dir", "/tmp/inventory_exports");
  
  db_path_ = this->get_parameter("database_path").as_string();
  auto_export_ = this->get_parameter("auto_export").as_bool();
  export_dir_ = this->get_parameter("export_dir").as_string();
  
  // Initialize database
  db_ = std::make_unique<DatabaseManager>(db_path_);
  
  // Start mission
  startNewMission();
  
  // Subscribers
  qr_sub_ = this->create_subscription<warehouse_rover_msgs::msg::QRDetectionArray>(
    "/qr_detections",
    10,
    std::bind(&InventoryNode::qrCallback, this, std::placeholders::_1)
  );
  
  RCLCPP_INFO(this->get_logger(), "Inventory Node started");
  RCLCPP_INFO(this->get_logger(), "Database: %s", db_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "Mission: %s", current_mission_id_.c_str());
}

InventoryNode::~InventoryNode()
{
  if (db_) {
    db_->endMission(current_mission_id_);
    
    if (auto_export_) {
      fs::create_directories(export_dir_);
      std::string export_path = export_dir_ + "/" + current_mission_id_ + ".json";
      db_->exportToJSON(current_mission_id_, export_path);
    }
  }
  
  RCLCPP_INFO(this->get_logger(), "Inventory Node shutdown");
}

void InventoryNode::startNewMission()
{
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << "MISSION_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
  current_mission_id_ = ss.str();
  
  db_->startMission(current_mission_id_);
}

void InventoryNode::qrCallback(const warehouse_rover_msgs::msg::QRDetectionArray::SharedPtr msg)
{
  if (msg->detections.empty()) {
    return;
  }
  
  for (const auto & detection : msg->detections) {
    // Only store valid detections
    if (!detection.is_valid) {
      RCLCPP_WARN(
        this->get_logger(),
        "Skipping invalid QR: \"%s\" - %s",
        detection.qr_data.c_str(),
        detection.error_message.c_str()
      );
      continue;
    }
    
    // Store in database
    int id = db_->addDetection(
      current_mission_id_,
      detection.rack_id,
      detection.shelf_id,
      detection.item_code,
      detection.confidence
    );
    
    if (id > 0) {
      RCLCPP_INFO(
        this->get_logger(),
        "✓ Stored [%s][%s] %s (ID: %d, conf: %.2f)",
        detection.rack_id.c_str(),
        detection.shelf_id.c_str(),
        detection.item_code.c_str(),
        id,
        detection.confidence
      );
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to store detection");
    }
  }
}

}  // namespace warehouse_rover_database

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<warehouse_rover_database::InventoryNode>(
    rclcpp::NodeOptions()
  );
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
