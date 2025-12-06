#ifndef WAREHOUSE_ROVER_QR_DETECTION__QR_PARSER_HPP_
#define WAREHOUSE_ROVER_QR_DETECTION__QR_PARSER_HPP_

#include <string>
#include <vector>

namespace warehouse_rover_qr_detection
{

/**
 * @brief Parsed QR code data
 */
struct ParsedQRData
{
  std::string rack_id;
  std::string shelf_id;
  std::string item_code;
  
  bool is_valid;
  std::string error_message;
  std::string raw_data;
};

/**
 * @brief QR Code Parser
 * 
 * Format: "RACKID_SHELFID_ITEMCODE"
 * Example: "R01_S2_ITM430"
 */
class QRParser
{
public:
  QRParser() = default;
  
  ParsedQRData parse(const std::string & qr_string) const;
  bool isValidFormat(const std::string & qr_string) const;

private:
  std::vector<std::string> split(const std::string & str, char delimiter) const;
  std::string trim(const std::string & str) const;
  bool isValidRackId(const std::string & rack_id) const;
  bool isValidShelfId(const std::string & shelf_id) const;
  bool isValidItemCode(const std::string & item_code) const;
};

}  // namespace warehouse_rover_qr_detection

#endif
