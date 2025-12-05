#include "warehouse_rover_image_processing/hardware_detector.hpp"
#include "warehouse_rover_image_processing/image_preprocessor.hpp"
#include <iostream>

int main(int, char**)
{
  std::cout << "Warehouse Rover - Hardware Detection Tool\n" << std::endl;
  
  warehouse_rover_image_processing::HardwareDetector detector;
  auto caps = detector.detect();
  
  detector.printCapabilities(caps);
  
  std::cout << "\nRecommended Configuration:" << std::endl;
  
  warehouse_rover_image_processing::ImagePreprocessor preprocessor(caps.recommended_mode);
  auto config = preprocessor.getRecommendedConfig();
  
  std::cout << "  Target Resolution: " << config.target_width << "x" << config.target_height << std::endl;
  std::cout << "  IPT Enabled: " << (config.enable_ipt ? "YES" : "NO") << std::endl;
  std::cout << "  Adaptive Threshold: " << (config.enable_adaptive ? "YES" : "NO") << std::endl;
  std::cout << "  Blur Kernel: " << config.blur_kernel_size << std::endl;
  std::cout << "  Interpolation: " << (config.interpolation == 1 ? "LINEAR" : 
                                       config.interpolation == 2 ? "CUBIC" : "NEAREST") << std::endl;
  
  return 0;
}
