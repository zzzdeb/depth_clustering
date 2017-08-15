#ifndef _SAVITSKY_GOLAY_SMOOTHING_
#define _SAVITSKY_GOLAY_SMOOTHING_


#include "utils/radians.h"
#include "utils/timer.h"
#include "projections/projection_params.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace depth_clustering{


class SavitskyGolaySmoothing
{

public:
    SavitskyGolaySmoothing() {}
    SavitskyGolaySmoothing(ProjectionParams params, int window_size)
    : 
    _params{params}, 
    _window_size{window_size} {}

    ~SavitskyGolaySmoothing() {}

    cv::Mat CreateAngleImage(const cv::Mat& depth_image);

  /**
   * @brief      Get kernel for Savitsky-Golay filter
   * @details    Get a column filter to process an image filled with data with
   *             Savitsky-Golay filter
   *
   * @param      window_size  size of the kernel
   * @return     column Mat kernel
   */
  cv::Mat GetSavitskyGolayKernel(int window_size) const;
  cv::Mat GetUniformKernel(int window_size, int type = CV_32F) const;

  /**
   * @brief      Apply Savitsky-Golay smoothing to a column
   * @param      column  [A column of an angle image]
   * @return     [a smoothed column]
   */

  cv::Mat ApplySavitskyGolaySmoothing(const cv::Mat& column, int window_size);
  /**
   * @brief      Get line angle
   * @details    Given two depth values and their angles compute the angle of
   *             the line that they spawn in the sensor frame.
   *
   * @param      depth_image  [32 bit float image]
   * @param      col          [current column]
   * @param      row_curr     [current row]
   * @param      row_neigh    [neighbor row]
   * @return     [angle of the line]
   */
  Radians GetLineAngle(const cv::Mat& depth_image, int col, int row_curr,
                       int row_neigh);

  /**
   * @brief      Repair zeros in the depth image
   *
   * @param[in]  depth_image  The depth image
   *
   * @return     depth image with repaired values
   */
  cv::Mat RepairDepth(const cv::Mat& no_ground_image, int step,
                      float depth_threshold);

  cv::Mat RepairDepth(const cv::Mat& depth_image);

  protected:
  ProjectionParams _params;
  int _window_size;
  float _eps = 0.001f;

};

}//end namespace

#endif
