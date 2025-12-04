#include "warehouse_rover_image_processing/hardware_detector.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>

#ifdef HAVE_OPENCV_CUDAARITHM
#include <opencv2/cudaarithm.hpp>
#endif

namespace warehouse_rover_image_processing
{

HardwareDetector::HardwareDetector()
{
}

bool HardwareDetector::isCudaAvailable()
{
#ifdef HAVE_OPENCV_CUDAARITHM
  return cv::cuda::getCudaEnabledDeviceCount() > 0;
#else
  return false;
#endif
}

bool HardwareDetector::isJetson()
{
  // Check for Jetson-specific files
  std::ifstream jetson_file("/etc/nv_tegra_release");
  if (jetson_file.good()) {
    return true;
  }
  
  // Check device tree
  std::ifstream model_file("/proc/device-tree/model");
  if (model_file.good()) {
    std::string model;
    std::getline(model_file, model);
    std::transform(model.begin(), model.end(), model.begin(), ::tolower);
    if (model.find("jetson") != std::string::npos) {
      return true;
    }
  }
  
  return false;
}

HardwareType HardwareDetector::detectGPUType(const std::string & gpu_name, size_t memory_mb)
{
  std::string name_lower = gpu_name;
  std::transform(name_lower.begin(), name_lower.end(), name_lower.begin(), ::tolower);
  
  // Jetson detection
  if (name_lower.find("orin") != std::string::npos) {
    return HardwareType::JETSON_ORIN_NANO;
  }
  if (name_lower.find("xavier") != std::string::npos) {
    return HardwareType::JETSON_XAVIER;
  }
  
  // High-end GPUs (RTX 40xx, 30xx, etc)
  if (name_lower.find("rtx 40") != std::string::npos ||
      name_lower.find("rtx 30") != std::string::npos ||
      memory_mb > 10000) {
    return HardwareType::NVIDIA_GPU_HIGH;
  }
  
  // Mid-range GPUs
  if (name_lower.find("rtx 20") != std::string::npos ||
      name_lower.find("gtx 16") != std::string::npos ||
      memory_mb > 6000) {
    return HardwareType::NVIDIA_GPU_MID;
  }
  
  // Low-end GPUs
  if (name_lower.find("gtx") != std::string::npos || memory_mb > 2000) {
    return HardwareType::NVIDIA_GPU_LOW;
  }
  
  return HardwareType::UNKNOWN;
}

HardwareCapabilities HardwareDetector::detect()
{
  HardwareCapabilities caps;
  caps.has_cuda = isCudaAvailable();
  
  if (!caps.has_cuda) {
    caps.type = HardwareType::CPU_ONLY;
    caps.cuda_device_count = 0;
    caps.recommended_mode = ProcessingMode::CPU_OPTIMIZED;
    return caps;
  }

#ifdef HAVE_OPENCV_CUDAARITHM
  caps.cuda_device_count = cv::cuda::getCudaEnabledDeviceCount();
  
  if (caps.cuda_device_count > 0) {
    cv::cuda::DeviceInfo dev_info(0);
    caps.gpu_name = dev_info.name();
    caps.gpu_memory_mb = dev_info.totalMemory() / (1024 * 1024);
    caps.compute_capability_major = dev_info.majorVersion();
    caps.compute_capability_minor = dev_info.minorVersion();
    
    // Estimate CUDA cores (approximate)
    int sm_count = dev_info.multiProcessorCount();
    if (caps.compute_capability_major == 8) {  // Ampere
      caps.cuda_cores = sm_count * 128;
    } else if (caps.compute_capability_major == 7) {  // Turing/Volta
      caps.cuda_cores = sm_count * 64;
    } else {
      caps.cuda_cores = sm_count * 128;  // Default estimate
    }
    
    // Detect hardware type
    if (isJetson()) {
      caps.type = detectGPUType(caps.gpu_name, caps.gpu_memory_mb);
      if (caps.type == HardwareType::UNKNOWN) {
        caps.type = HardwareType::JETSON_ORIN_NANO;  // Default Jetson
      }
    } else {
      caps.type = detectGPUType(caps.gpu_name, caps.gpu_memory_mb);
    }
    
    caps.recommended_mode = selectProcessingMode(caps);
  }
#endif
  
  return caps;
}

ProcessingMode HardwareDetector::selectProcessingMode(const HardwareCapabilities & hw)
{
  switch (hw.type) {
    case HardwareType::NVIDIA_GPU_HIGH:  // RTX 4070
      return ProcessingMode::CUDA_AGGRESSIVE;
      
    case HardwareType::JETSON_ORIN_NANO:
      return ProcessingMode::CUDA_OPTIMIZED;
      
    case HardwareType::JETSON_XAVIER:
      return ProcessingMode::CUDA_OPTIMIZED;
      
    case HardwareType::NVIDIA_GPU_MID:
      return ProcessingMode::CUDA_OPTIMIZED;
      
    case HardwareType::NVIDIA_GPU_LOW:
      return ProcessingMode::CUDA_BASIC;
      
    case HardwareType::CPU_ONLY:
      return ProcessingMode::CPU_OPTIMIZED;
      
    default:
      return ProcessingMode::CPU_BASIC;
  }
}

void HardwareDetector::printCapabilities(const HardwareCapabilities & hw)
{
  std::cout << "\n=== Hardware Capabilities ===" << std::endl;
  std::cout << "CUDA Available: " << (hw.has_cuda ? "YES" : "NO") << std::endl;
  
  if (hw.has_cuda) {
    std::cout << "GPU Name: " << hw.gpu_name << std::endl;
    std::cout << "GPU Memory: " << hw.gpu_memory_mb << " MB" << std::endl;
    std::cout << "CUDA Cores: ~" << hw.cuda_cores << std::endl;
    std::cout << "Compute Capability: " << hw.compute_capability_major 
              << "." << hw.compute_capability_minor << std::endl;
    std::cout << "Hardware Type: ";
    
    switch (hw.type) {
      case HardwareType::NVIDIA_GPU_HIGH:
        std::cout << "High-End NVIDIA GPU" << std::endl;
        break;
      case HardwareType::JETSON_ORIN_NANO:
        std::cout << "Jetson Orin Nano" << std::endl;
        break;
      case HardwareType::JETSON_XAVIER:
        std::cout << "Jetson Xavier" << std::endl;
        break;
      case HardwareType::NVIDIA_GPU_MID:
        std::cout << "Mid-Range NVIDIA GPU" << std::endl;
        break;
      default:
        std::cout << "Other CUDA Device" << std::endl;
    }
    
    std::cout << "Recommended Mode: ";
    switch (hw.recommended_mode) {
      case ProcessingMode::CUDA_AGGRESSIVE:
        std::cout << "CUDA Aggressive (Full Power)" << std::endl;
        break;
      case ProcessingMode::CUDA_OPTIMIZED:
        std::cout << "CUDA Optimized (Balanced)" << std::endl;
        break;
      case ProcessingMode::CUDA_BASIC:
        std::cout << "CUDA Basic" << std::endl;
        break;
      default:
        std::cout << "CPU Fallback" << std::endl;
    }
  } else {
    std::cout << "Hardware Type: CPU Only" << std::endl;
    std::cout << "Recommended Mode: CPU Optimized" << std::endl;
  }
  std::cout << "============================\n" << std::endl;
}

}  // namespace warehouse_rover_image_processing
