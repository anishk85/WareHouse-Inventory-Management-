#ifndef WAREHOUSE_ROVER_DATABASE__DATABASE_MANAGER_HPP_
#define WAREHOUSE_ROVER_DATABASE__DATABASE_MANAGER_HPP_

#include <sqlite3.h>
#include <string>
#include <vector>

namespace warehouse_rover_database
{

struct Detection {
  int id;
  std::string mission_id;
  std::string rack_id;
  std::string shelf_id;
  std::string item_code;
  std::string timestamp;
  double confidence;
};

struct MissionSummary {
  std::string mission_id;
  std::string start_time;
  std::string end_time;
  std::string status;
  int total_detections;
};

class DatabaseManager
{
public:
  explicit DatabaseManager(const std::string & db_path);
  ~DatabaseManager();
  
  // Mission management
  bool startMission(const std::string & mission_id);
  bool endMission(const std::string & mission_id);
  MissionSummary getMissionSummary(const std::string & mission_id);
  
  // Detection storage
  int addDetection(
    const std::string & mission_id,
    const std::string & rack_id,
    const std::string & shelf_id,
    const std::string & item_code,
    double confidence
  );
  
  std::vector<Detection> getDetectionsByRack(const std::string & rack_id);
  std::vector<Detection> getDetectionsByMission(const std::string & mission_id);
  
  // Export
  bool exportToJSON(const std::string & mission_id, const std::string & output_path);

private:
  void initializeDatabase();
  std::string getCurrentTimestamp();
  bool executeSQL(const std::string & sql);
  
  sqlite3 * db_;
  std::string db_path_;
};

}  // namespace warehouse_rover_database

#endif
