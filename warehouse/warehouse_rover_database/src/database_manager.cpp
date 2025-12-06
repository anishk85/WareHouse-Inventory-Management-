#include "warehouse_rover_database/database_manager.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <fstream>

namespace warehouse_rover_database
{

DatabaseManager::DatabaseManager(const std::string & db_path)
: db_(nullptr), db_path_(db_path)
{
  int rc = sqlite3_open(db_path.c_str(), &db_);
  if (rc != SQLITE_OK) {
    std::cerr << "Cannot open database: " << sqlite3_errmsg(db_) << std::endl;
    return;
  }
  
  std::cout << "Database opened: " << db_path << std::endl;
  initializeDatabase();
}

DatabaseManager::~DatabaseManager()
{
  if (db_) {
    sqlite3_close(db_);
  }
}

void DatabaseManager::initializeDatabase()
{
  // Missions table
  executeSQL(R"(
    CREATE TABLE IF NOT EXISTS missions (
      mission_id TEXT PRIMARY KEY,
      start_time TEXT NOT NULL,
      end_time TEXT,
      status TEXT DEFAULT 'in_progress',
      total_detections INTEGER DEFAULT 0
    )
  )");
  
  // Detections table
  executeSQL(R"(
    CREATE TABLE IF NOT EXISTS detections (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      mission_id TEXT NOT NULL,
      rack_id TEXT NOT NULL,
      shelf_id TEXT NOT NULL,
      item_code TEXT NOT NULL,
      timestamp TEXT NOT NULL,
      confidence REAL,
      FOREIGN KEY (mission_id) REFERENCES missions(mission_id)
    )
  )");
  
  // Index for fast queries
  executeSQL("CREATE INDEX IF NOT EXISTS idx_rack ON detections(rack_id)");
  executeSQL("CREATE INDEX IF NOT EXISTS idx_mission ON detections(mission_id)");
  
  std::cout << "Database initialized" << std::endl;
}

bool DatabaseManager::executeSQL(const std::string & sql)
{
  char * err_msg = nullptr;
  int rc = sqlite3_exec(db_, sql.c_str(), nullptr, nullptr, &err_msg);
  
  if (rc != SQLITE_OK) {
    std::cerr << "SQL error: " << err_msg << std::endl;
    sqlite3_free(err_msg);
    return false;
  }
  
  return true;
}

std::string DatabaseManager::getCurrentTimestamp()
{
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
  return ss.str();
}

bool DatabaseManager::startMission(const std::string & mission_id)
{
  std::string timestamp = getCurrentTimestamp();
  
  std::stringstream sql;
  sql << "INSERT INTO missions (mission_id, start_time, status) "
      << "VALUES ('" << mission_id << "', '" << timestamp << "', 'in_progress')";
  
  bool success = executeSQL(sql.str());
  if (success) {
    std::cout << "Mission started: " << mission_id << std::endl;
  }
  return success;
}

bool DatabaseManager::endMission(const std::string & mission_id)
{
  std::string timestamp = getCurrentTimestamp();
  
  // Get detection count
  std::stringstream count_sql;
  count_sql << "SELECT COUNT(*) FROM detections WHERE mission_id = '" << mission_id << "'";
  
  sqlite3_stmt * stmt;
  int total_detections = 0;
  
  if (sqlite3_prepare_v2(db_, count_sql.str().c_str(), -1, &stmt, nullptr) == SQLITE_OK) {
    if (sqlite3_step(stmt) == SQLITE_ROW) {
      total_detections = sqlite3_column_int(stmt, 0);
    }
    sqlite3_finalize(stmt);
  }
  
  // Update mission
  std::stringstream sql;
  sql << "UPDATE missions SET "
      << "end_time = '" << timestamp << "', "
      << "status = 'completed', "
      << "total_detections = " << total_detections << " "
      << "WHERE mission_id = '" << mission_id << "'";
  
  bool success = executeSQL(sql.str());
  if (success) {
    std::cout << "Mission ended: " << mission_id 
              << " (Detections: " << total_detections << ")" << std::endl;
  }
  return success;
}

int DatabaseManager::addDetection(
  const std::string & mission_id,
  const std::string & rack_id,
  const std::string & shelf_id,
  const std::string & item_code,
  double confidence)
{
  std::string timestamp = getCurrentTimestamp();
  
  std::stringstream sql;
  sql << "INSERT INTO detections "
      << "(mission_id, rack_id, shelf_id, item_code, timestamp, confidence) "
      << "VALUES ("
      << "'" << mission_id << "', "
      << "'" << rack_id << "', "
      << "'" << shelf_id << "', "
      << "'" << item_code << "', "
      << "'" << timestamp << "', "
      << confidence << ")";
  
  if (executeSQL(sql.str())) {
    return sqlite3_last_insert_rowid(db_);
  }
  
  return -1;
}

std::vector<Detection> DatabaseManager::getDetectionsByRack(const std::string & rack_id)
{
  std::vector<Detection> results;
  
  std::stringstream sql;
  sql << "SELECT id, mission_id, rack_id, shelf_id, item_code, timestamp, confidence "
      << "FROM detections WHERE rack_id = '" << rack_id << "' "
      << "ORDER BY timestamp DESC";
  
  sqlite3_stmt * stmt;
  if (sqlite3_prepare_v2(db_, sql.str().c_str(), -1, &stmt, nullptr) == SQLITE_OK) {
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      Detection det;
      det.id = sqlite3_column_int(stmt, 0);
      det.mission_id = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
      det.rack_id = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
      det.shelf_id = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3));
      det.item_code = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4));
      det.timestamp = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 5));
      det.confidence = sqlite3_column_double(stmt, 6);
      
      results.push_back(det);
    }
    sqlite3_finalize(stmt);
  }
  
  return results;
}

MissionSummary DatabaseManager::getMissionSummary(const std::string & mission_id)
{
  MissionSummary summary;
  
  std::stringstream sql;
  sql << "SELECT mission_id, start_time, end_time, status, total_detections "
      << "FROM missions WHERE mission_id = '" << mission_id << "'";
  
  sqlite3_stmt * stmt;
  if (sqlite3_prepare_v2(db_, sql.str().c_str(), -1, &stmt, nullptr) == SQLITE_OK) {
    if (sqlite3_step(stmt) == SQLITE_ROW) {
      summary.mission_id = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
      summary.start_time = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
      
      const char* end_time = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
      summary.end_time = end_time ? end_time : "";
      
      summary.status = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3));
      summary.total_detections = sqlite3_column_int(stmt, 4);
    }
    sqlite3_finalize(stmt);
  }
  
  return summary;
}

bool DatabaseManager::exportToJSON(const std::string & mission_id, const std::string & output_path)
{
  auto summary = getMissionSummary(mission_id);
  
  std::ofstream file(output_path);
  if (!file.is_open()) {
    return false;
  }
  
  file << "{\n";
  file << "  \"mission_id\": \"" << summary.mission_id << "\",\n";
  file << "  \"start_time\": \"" << summary.start_time << "\",\n";
  file << "  \"end_time\": \"" << summary.end_time << "\",\n";
  file << "  \"status\": \"" << summary.status << "\",\n";
  file << "  \"total_detections\": " << summary.total_detections << ",\n";
  file << "  \"detections\": [\n";
  
  // Get detections
  auto detections = getDetectionsByMission(mission_id);
  for (size_t i = 0; i < detections.size(); ++i) {
    const auto & det = detections[i];
    file << "    {\n";
    file << "      \"rack_id\": \"" << det.rack_id << "\",\n";
    file << "      \"shelf_id\": \"" << det.shelf_id << "\",\n";
    file << "      \"item_code\": \"" << det.item_code << "\",\n";
    file << "      \"timestamp\": \"" << det.timestamp << "\",\n";
    file << "      \"confidence\": " << det.confidence << "\n";
    file << "    }";
    if (i < detections.size() - 1) file << ",";
    file << "\n";
  }
  
  file << "  ]\n";
  file << "}\n";
  file.close();
  
  std::cout << "Exported to: " << output_path << std::endl;
  return true;
}

std::vector<Detection> DatabaseManager::getDetectionsByMission(const std::string & mission_id)
{
  std::vector<Detection> results;
  
  std::stringstream sql;
  sql << "SELECT id, mission_id, rack_id, shelf_id, item_code, timestamp, confidence "
      << "FROM detections WHERE mission_id = '" << mission_id << "' "
      << "ORDER BY id";
  
  sqlite3_stmt * stmt;
  if (sqlite3_prepare_v2(db_, sql.str().c_str(), -1, &stmt, nullptr) == SQLITE_OK) {
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      Detection det;
      det.id = sqlite3_column_int(stmt, 0);
      det.mission_id = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
      det.rack_id = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
      det.shelf_id = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3));
      det.item_code = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4));
      det.timestamp = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 5));
      det.confidence = sqlite3_column_double(stmt, 6);
      
      results.push_back(det);
    }
    sqlite3_finalize(stmt);
  }
  
  return results;
}

}  // namespace warehouse_rover_database
