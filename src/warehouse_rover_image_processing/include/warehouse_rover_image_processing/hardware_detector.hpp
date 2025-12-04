#ifndef WAREHOUSE_ROVER_IMAGE_PROCESSING__HARDWARE_DETECTOR_HPP_
#define WAREHOUSE_ROVER_IMAGE_PROCESSING__HARDWARE_DETECTOR_HPP_

#include <opencv2/opencv.hpp>
#include <string>

namespace warehouse_rover_image_processing
{

enum class HardwareType {
  UNKNOWN,
  CPU_ONLY,
  JETSON_ORIN_NANO,
  JETSON_XAVIER,
  NVIDIA_GPU_HIGH,    // RTX 4070, etc
  NVIDIA_GPU_MID,
  NVIDIA_GPU_LOW
};

enum class ProcessingMode {
  CPU_BASIC,
  CPU_OPTIMIZED,
  CUDA_BASIC,
  CUDA_OPTIMIZED,
  CUDA_AGGRESSIVE
};

struct HardwareCapabilities {
  HardwareType type;
  bool has_cuda;
  int cuda_device_count;
  std::string gpu_name;
  size_t gpu_memory_mb;
  int compute_capability_major;
  int compute_capability_minor;
  int cuda_cores;
  ProcessingMode recommended_mode;
};

class HardwareDetector
{
public:
  HardwareDetector();
  
  HardwareCapabilities detect();
  ProcessingMode selectProcessingMode(const HardwareCapabilities & hw);
  
  void printCapabilities(const HardwareCapabilities & hw);

private:
  bool isCudaAvailable();
  bool isJetson();
  HardwareType detectGPUType(const std::string & gpu_name, size_t memory_mb);
};

}  // namespace warehouse_rover_image_processing

#endif
