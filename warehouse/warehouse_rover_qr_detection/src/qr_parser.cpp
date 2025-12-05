#include "warehouse_rover_qr_detection/qr_parser.hpp"
#include <algorithm>
#include <sstream>
#include <regex>

namespace warehouse_rover_qr_detection
{

ParsedQRData QRParser::parse(const std::string & qr_string) const
{
  ParsedQRData result;
  result.raw_data = qr_string;
  result.is_valid = false;
  
  std::string trimmed = trim(qr_string);
  
  if (trimmed.empty()) {
    result.error_message = "Empty QR code";
    return result;
  }
  
  // Split: "R01_S2_ITM430" -> ["R01", "S2", "ITM430"]
  auto parts = split(trimmed, '_');
  
  if (parts.size() != 3) {
    result.error_message = "Invalid format. Expected: RACKID_SHELFID_ITEMCODE";
    return result;
  }
  
  std::string rack_id = trim(parts[0]);
  std::string shelf_id = trim(parts[1]);
  std::string item_code = trim(parts[2]);
  
  if (!isValidRackId(rack_id)) {
    result.error_message = "Invalid rack ID: " + rack_id;
    return result;
  }
  
  if (!isValidShelfId(shelf_id)) {
    result.error_message = "Invalid shelf ID: " + shelf_id;
    return result;
  }
  
  if (!isValidItemCode(item_code)) {
    result.error_message = "Invalid item code: " + item_code;
    return result;
  }
  
  result.rack_id = rack_id;
  result.shelf_id = shelf_id;
  result.item_code = item_code;
  result.is_valid = true;
  
  return result;
}

bool QRParser::isValidFormat(const std::string & qr_string) const
{
  return parse(qr_string).is_valid;
}

std::vector<std::string> QRParser::split(const std::string & str, char delimiter) const
{
  std::vector<std::string> tokens;
  std::stringstream ss(str);
  std::string token;
  
  while (std::getline(ss, token, delimiter)) {
    tokens.push_back(token);
  }
  
  return tokens;
}

std::string QRParser::trim(const std::string & str) const
{
  auto start = std::find_if_not(str.begin(), str.end(), ::isspace);
  auto end = std::find_if_not(str.rbegin(), str.rend(), ::isspace).base();
  return (start < end) ? std::string(start, end) : std::string();
}

bool QRParser::isValidRackId(const std::string & rack_id) const
{
  // Format: R + digits (R01, R02, R10)
  std::regex pattern("^R\\d+$");
  return std::regex_match(rack_id, pattern);
}

bool QRParser::isValidShelfId(const std::string & shelf_id) const
{
  // Format: S + digits (S1, S2, S5)
  std::regex pattern("^S\\d+$");
  return std::regex_match(shelf_id, pattern);
}

bool QRParser::isValidItemCode(const std::string & item_code) const
{
  if (item_code.empty()) return false;
  
  // Allow alphanumeric, underscore, hyphen
  std::regex pattern("^[A-Za-z0-9_-]+$");
  return std::regex_match(item_code, pattern);
}

}  // namespace warehouse_rover_qr_detection
