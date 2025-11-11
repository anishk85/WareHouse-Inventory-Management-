#include "warehouse_rover_image_processing/image_preprocessor.hpp"
#include <iostream>

namespace warehouse_rover_image_processing
{

ImagePreprocessor::ImagePreprocessor(ProcessingMode mode)
: mode_(mode)
{
#ifdef HAVE_OPENCV_CUDAARITHM
  if (mode_ != ProcessingMode::CPU_BASIC && mode_ != ProcessingMode::CPU_OPTIMIZED) {
    // Pre-create GPU filter for reuse
    gaussian_filter_ = cv::cuda::createGaussianFilter(
      CV_8UC1, CV_8UC1, cv::Size(3, 3), 0);
    
    std::cout << "GPU preprocessing initialized" << std::endl;
  }
#endif
}

ImagePreprocessor::~ImagePreprocessor()
{
}

PreprocessingConfig ImagePreprocessor::getRecommendedConfig()
{
  PreprocessingConfig config;
  
  switch (mode_) {
    case ProcessingMode::CUDA_AGGRESSIVE:  // RTX 4070
      config.target_width = 1920;
      config.target_height = 1080;
      config.enable_ipt = true;
      config.enable_adaptive = true;
      config.blur_kernel_size = 3;
      config.adaptive_block_size = 11;
      config.interpolation = cv::INTER_LINEAR;
      break;
      
    case ProcessingMode::CUDA_OPTIMIZED:  // Jetson Orin Nano
      config.target_width = 1280;
      config.target_height = 720;
      config.enable_ipt = true;
      config.enable_adaptive = true;
      config.blur_kernel_size = 3;
      config.adaptive_block_size = 11;
      config.interpolation = cv::INTER_LINEAR;
      break;
      
    case ProcessingMode::CUDA_BASIC:
      config.target_width = 960;
      config.target_height = 540;
      config.enable_ipt = true;
      config.enable_adaptive = false;
      config.blur_kernel_size = 3;
      config.adaptive_block_size = 11;
      config.interpolation = cv::INTER_NEAREST;
      break;
      
    case ProcessingMode::CPU_OPTIMIZED:
      config.target_width = 1280;
      config.target_height = 720;
      config.enable_ipt = true;
      config.enable_adaptive = true;
      config.blur_kernel_size = 3;
      config.adaptive_block_size = 11;
      config.interpolation = cv::INTER_LINEAR;
      break;
      
    default:  // CPU_BASIC
      config.target_width = 640;
      config.target_height = 480;
      config.enable_ipt = false;
      config.enable_adaptive = false;
      config.blur_kernel_size = 3;
      config.adaptive_block_size = 11;
      config.interpolation = cv::INTER_NEAREST;
  }
  
  return config;
}

cv::Mat ImagePreprocessor::process(const cv::Mat & input, PreprocessingConfig config)
{
#ifdef HAVE_OPENCV_CUDAARITHM
  if (mode_ != ProcessingMode::CPU_BASIC && mode_ != ProcessingMode::CPU_OPTIMIZED) {
    return processCUDA(input, config);
  }
#endif
  return processCPU(input, config);
}

cv::Mat ImagePreprocessor::toGrayscale(const cv::Mat & input)
{
  cv::Mat gray;
  if (input.channels() == 3) {
    cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = input.clone();
  }
  return gray;
}

cv::Mat ImagePreprocessor::gaussianBlur(const cv::Mat & input, int kernel_size)
{
  cv::Mat blurred;
  if (kernel_size > 1) {
    cv::GaussianBlur(input, blurred, cv::Size(kernel_size, kernel_size), 0);
  } else {
    blurred = input.clone();
  }
  return blurred;
}

cv::Mat ImagePreprocessor::adaptiveThreshold(const cv::Mat & input, int block_size)
{
  cv::Mat thresh;
  cv::adaptiveThreshold(input, thresh, 255,
                        cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                        cv::THRESH_BINARY,
                        block_size, 2);
  return thresh;
}

cv::Mat ImagePreprocessor::processCPU(const cv::Mat & input, PreprocessingConfig config)
{
  cv::Mat result = input.clone();
  
  // Resize if needed
  if (input.cols != config.target_width || input.rows != config.target_height) {
    cv::resize(result, result, 
               cv::Size(config.target_width, config.target_height),
               0, 0, cv::INTER_AREA);
  }
  
  // Convert to grayscale
  if (result.channels() == 3) {
    cv::cvtColor(result, result, cv::COLOR_BGR2GRAY);
  }
  
  // Gaussian blur
  if (config.blur_kernel_size > 1) {
    cv::GaussianBlur(result, result, 
                     cv::Size(config.blur_kernel_size, config.blur_kernel_size), 0);
  }
  
  // Adaptive thresholding
  if (config.enable_adaptive) {
    cv::adaptiveThreshold(result, result, 255,
                          cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY,
                          config.adaptive_block_size, 2);
  }
  
  return result;
}

#ifdef HAVE_OPENCV_CUDAARITHM
cv::Mat ImagePreprocessor::processCUDA(const cv::Mat & input, PreprocessingConfig config)
{
  // Upload to GPU (reuse buffer)
  gpu_input_.upload(input);
  
  // Resize on GPU if needed
  if (input.cols != config.target_width || input.rows != config.target_height) {
    cv::cuda::resize(gpu_input_, gpu_input_,
                     cv::Size(config.target_width, config.target_height),
                     0, 0, cv::INTER_AREA, stream1_);
  }
  
  // Convert to grayscale on GPU
  if (gpu_input_.channels() == 3) {
    cv::cuda::cvtColor(gpu_input_, gpu_gray_, cv::COLOR_BGR2GRAY, 0, stream1_);
  } else {
    gpu_gray_ = gpu_input_;
  }
  
  // Gaussian blur on GPU
  if (config.blur_kernel_size > 1) {
    if (!gaussian_filter_ || gaussian_filter_->ksize() != cv::Size(config.blur_kernel_size, config.blur_kernel_size)) {
      gaussian_filter_ = cv::cuda::createGaussianFilter(
        CV_8UC1, CV_8UC1, 
        cv::Size(config.blur_kernel_size, config.blur_kernel_size), 0);
    }
    gaussian_filter_->apply(gpu_gray_, gpu_blurred_, stream1_);
  } else {
    gpu_blurred_ = gpu_gray_;
  }
  
  // Download result
  cv::Mat result;
  if (config.enable_adaptive) {
    gpu_blurred_.download(result, stream1_);
    stream1_.waitForCompletion();
    
    cv::adaptiveThreshold(result, result, 255,
                          cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY,
                          config.adaptive_block_size, 2);
  } else {
    gpu_blurred_.download(result, stream1_);
    stream1_.waitForCompletion();
  }
  
  return result;
}
#endif

bool ImagePreprocessor::detectCorners(const cv::Mat & input, std::vector<cv::Point2f> & corners)
{
  cv::QRCodeDetector detector;
  cv::Mat gray;
  
  if (input.channels() == 3) {
    cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = input;
  }
  
  std::vector<cv::Point> points;
  bool detected = detector.detect(gray, points);
  
  if (detected && points.size() >= 4) {
    corners.clear();
    for (const auto & pt : points) {
      corners.push_back(cv::Point2f(pt.x, pt.y));
    }
    return true;
  }
  
  return false;
}

cv::Mat ImagePreprocessor::applyIPT(const cv::Mat & input, const std::vector<cv::Point2f> & corners)
{
  if (corners.size() != 4) {
    return input;
  }
  
  // Define target corners (perfect square)
  int size = 300;
  std::vector<cv::Point2f> target_corners = {
    cv::Point2f(0, 0),
    cv::Point2f(size, 0),
    cv::Point2f(size, size),
    cv::Point2f(0, size)
  };
  
  // Calculate homography
  cv::Mat H = cv::getPerspectiveTransform(corners, target_corners);
  
  // Apply perspective transform
  cv::Mat warped;
  
#ifdef HAVE_OPENCV_CUDAARITHM
  if (mode_ != ProcessingMode::CPU_BASIC && mode_ != ProcessingMode::CPU_OPTIMIZED) {
    gpu_input_.upload(input);
    cv::cuda::warpPerspective(gpu_input_, gpu_warped_, H, cv::Size(size, size),
                              cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                              cv::Scalar(), stream1_);
    gpu_warped_.download(warped, stream1_);
    stream1_.waitForCompletion();
  } else
#endif
  {
    cv::warpPerspective(input, warped, H, cv::Size(size, size), cv::INTER_LINEAR);
  }
  
  return warped;
}

}  // namespace warehouse_rover_image_processing
