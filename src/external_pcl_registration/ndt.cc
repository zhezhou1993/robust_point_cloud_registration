#include "external_pcl_registration/ndt.h"

#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>


Ndt::Ndt(const NdtParameters& params)
  : params_(params) {}

Eigen::Affine3f Ndt::evaluate(
    pcl::PointCloud<PointType>::Ptr source_cloud,
    pcl::PointCloud<PointType>::Ptr target_cloud) {
  CHECK(source_cloud);
  CHECK(target_cloud);

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setInputSource(source_cloud);
  ndt.setInputTarget(target_cloud);

  if (!params_.use_default_parameters) {
    ndt.setTransformationEpsilon(params_.transformation_epsilon);
    ndt.setStepSize(params_.step_size);
    ndt.setResolution(params_.resolution);
    ndt.setMaximumIterations(params_.maximum_iterations);
    ndt.setMaxCorrespondenceDistance(params_.maximum_correspondence_distance);
  }
  LOG(INFO) << "MaxCorrespondenceDistance: " << ndt.getMaxCorrespondenceDistance();
  LOG(INFO) << "RANSACOutlierRejectionThreshold: " << ndt.getRANSACOutlierRejectionThreshold();
  LOG(INFO) << "TransformationEpsilon" << ndt.getTransformationEpsilon();
  LOG(INFO) << "StepSize: " << ndt.getStepSize();
  LOG(INFO) << "Resolution: " << ndt.getResolution();
  LOG(INFO) << "MaximumIterations: " << ndt.getMaximumIterations();

  pcl::PointCloud<PointType>::Ptr aligned_source =
      boost::make_shared<pcl::PointCloud<PointType>>();
  ndt.align(*aligned_source);
  LOG(INFO) << "Final transformation: " << std::endl << ndt.getFinalTransformation();
  final_transform = ndt.getFinalTransformation();
  final_trans_affine = final_transform.matrix();

  if (ndt.hasConverged()) {
    LOG(INFO) << "NDT converged." << std::endl
              << "The score is " << ndt.getFitnessScore();
  } else {
    LOG(INFO) << "NDT did not converge.";
  }

  if (params_.save_aligned_cloud) {
    LOG(INFO) << "Saving aligned source cloud to: " << params_.aligned_cloud_filename;
    pcl::io::savePCDFile(params_.aligned_cloud_filename, *aligned_source);
  }

  if (params_.visualize_clouds) {
    source_cloud->header.frame_id = params_.frame_id;
    target_cloud->header.frame_id = params_.frame_id;
    aligned_source->header.frame_id = params_.frame_id;
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer
        (new pcl::visualization::PCLVisualizer ("NDT: source(red), target(green), aligned(blue)"));
    viewer->setBackgroundColor(255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        source_cloud_handler(source_cloud, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        target_cloud_handler(target_cloud, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        aligned_source_handler(aligned_source, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ>(source_cloud, source_cloud_handler, "source");
    viewer->addPointCloud<pcl::PointXYZ>(target_cloud, target_cloud_handler, "target");
    viewer->addPointCloud<pcl::PointXYZ>(aligned_source, aligned_source_handler, "aligned source");
    viewer->spin();
  }

  return final_trans_affine;
}
