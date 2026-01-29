#ifndef WAREHOUSE_ROVER_IMAGE_PROCESSING__IMAGE_PREPROCESSOR_HPP_
#define WAREHOUSE_ROVER_IMAGE_PROCESSING__IMAGE_PREPROCESSOR_HPP_

#include <opencv2/opencv.hpp>
#include "warehouse_rover_image_processing/hardware_detector.hpp"

#ifdef HAVE_OPENCV_CUDAARITHM
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudafilters.hpp>
#endif

namespace warehouse_rover_image_processing
{

struct PreprocessingConfig {
  int target_width;          // Target resolution width
  int target_height;         // Target resolution height
  bool enable_ipt;           // Inverse Perspective Transform
  bool enable_adaptive;      // Adaptive thresholding
  int blur_kernel_size;      // Gaussian blur kernel
  int adaptive_block_size;   // Adaptive threshold block size
  int interpolation;         // Interpolation method
};

class ImagePreprocessor
{
public:
  ImagePreprocessor(ProcessingMode mode);
  ~ImagePreprocessor();
  
  // Main preprocessing pipeline
  cv::Mat process(const cv::Mat & input, PreprocessingConfig config);
  
  // Individual stages (for testing)
  cv::Mat toGrayscale(const cv::Mat & input);
  cv::Mat gaussianBlur(const cv::Mat & input, int kernel_size);
  cv::Mat adaptiveThreshold(const cv::Mat & input, int block_size);
  cv::Mat applyIPT(const cv::Mat & input, const std::vector<cv::Point2f> & corners);
  
  // Corner detection
  bool detectCorners(const cv::Mat & input, std::vector<cv::Point2f> & corners);
  
  // Get recommended config for current hardware
  PreprocessingConfig getRecommendedConfig();
  
  ProcessingMode getMode() const { return mode_; }

private:
  ProcessingMode mode_;
  
#ifdef HAVE_OPENCV_CUDAARITHM
  // GPU buffers (reused to avoid allocation overhead)
  cv::cuda::GpuMat gpu_input_;
  cv::cuda::GpuMat gpu_gray_;
  cv::cuda::GpuMat gpu_blurred_;
  cv::cuda::GpuMat gpu_thresh_;
  cv::cuda::GpuMat gpu_warped_;
  cv::cuda::Stream stream1_;
  cv::cuda::Stream stream2_;
  cv::Ptr<cv::cuda::Filter> gaussian_filter_;
#endif
  
  // CPU processing
  cv::Mat processCPU(const cv::Mat & input, PreprocessingConfig config);
  
#ifdef HAVE_OPENCV_CUDAARITHM
  // GPU processing
  cv::Mat processCUDA(const cv::Mat & input, PreprocessingConfig config);
#endif
};

}  // namespace warehouse_rover_image_processing

#endif
