#ifndef GL_DEPTH_SIM_PCL_INTERFACE_H
#define GL_DEPTH_SIM_PCL_INTERFACE_H

#include "gl_depth_sim/camera_properties.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace gl_depth_sim
{

/**
 * @brief Projects the given depth image forward into space using the camera intrinsics from @e camera. The
 * resulting point cloud is organized (has width and height) and NOT dense. Invalid points are marked as NAN.
 */
void toPointCloudXYZ(const CameraProperties& camera, const DepthImage& depth, pcl::PointCloud<pcl::PointXYZ>& out);

/**
 * @brief Projects the given depth image forward into space using the camera intrinsics from @e camera. The
 * resulting point cloud is organized (has width and height) and NOT dense. Invalid points are marked as NAN.
 */
template <typename PointT>
void toPointCloud(const gl_depth_sim::CameraProperties& camera, const gl_depth_sim::DepthImage& depth,
                  pcl::PointCloud<PointT>& out)
{
  out.width = depth.cols;
  out.height = depth.rows;
  out.resize(out.width * out.height);
  out.is_dense = false;

  for (int i = 0; i < depth.rows; ++i)
  {
    for (int j = 0; j < depth.cols; ++j)
    {
      const float distance = depth.distance(i, j);
      PointT& pt = out(j, i);

      if (distance != 0.0f)
      {
        pt.z = distance;
        pt.x = (j - camera.cx) * distance / camera.fx;
        pt.y = (i - camera.cy) * distance / camera.fy;
      }
      else
      {
        pt.z = pt.x = pt.y = std::numeric_limits<float>::quiet_NaN();
      }
    }
  } // end of loop
}

}

#endif
