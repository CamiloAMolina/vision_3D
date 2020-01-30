#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
//#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>

//#include <fstream>
#include <iostream>

//typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointXYZ PointType;

int main (int argc, char** argv)
{
   pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
   pcl::io::loadPCDFile ("tuerca_scale_ascii.pcd", *cloud);
   
   pcl::PointCloud<PointType>::Ptr cloud_keypoints (new pcl::PointCloud<PointType> ());
   pcl::PointCloud<pcl::SHOT352>::Ptr cloud_descriptors (new pcl::PointCloud<pcl::SHOT352> ()); //Descriptor Cloud PCL
   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
   
// Create the normal estimation class, and pass the input dataset to it
   pcl::NormalEstimationOMP<PointType, pcl::Normal> norm_est;
   norm_est.setInputCloud (cloud);


// Create an empty kdtree representation, and pass it to the normal estimation object.
// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  //norm_est.setSearchMethod (tree);

  //ne.setRadiusSearch (0.0001);  //Use all neighbors in a sphere of radius 3cm
  norm_est.setKSearch (10);
  norm_est.compute (*cloud_normals);


   pcl::PCDWriter writer;
   writer.write("cloud_normalsOMP.pcd", *cloud_normals, false);
  
  pcl::UniformSampling<PointType> uniform_sampling;
  uniform_sampling.setInputCloud (cloud);
  uniform_sampling.setRadiusSearch (0.01f);
  uniform_sampling.filter (*cloud_keypoints);
  std::cout << "Model total points: " << cloud->size () << "; Selected Keypoints: " << cloud_keypoints->size () << std::endl;
  
    pcl::PCDWriter writer1;
  writer1.write("cloud_keypoints.pcd", *cloud_keypoints, false);

  //
  //  Compute Descriptor for keypoints
  //
  pcl::SHOTEstimationOMP<PointType, pcl::Normal, pcl::SHOT352> descr_est;
  descr_est.setRadiusSearch (0.02f);

  descr_est.setInputCloud (cloud_keypoints);
  descr_est.setInputNormals (cloud_normals);
  descr_est.setSearchSurface (cloud);
  descr_est.compute (*cloud_descriptors);
  
  //pcl::PCDWriter writer1;
  writer1.write("descriptor.pcd", *cloud_descriptors, false);

  

//_____________________________________________________________________________



pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
viewer.setBackgroundColor (255, 255, 255);

  pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

    //  We are translating the model so that it doesn't end in the middle of the scene representation
   pcl::transformPointCloud (*cloud, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
   pcl::transformPointCloud (*cloud_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 0, 0, 0);
    viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "off_scene_model");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints,  255, 0, 0);
    viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "off_scene_model_keypoints");
    
    pcl::visualization::PCLVisualizer viewer_nor ("Normals");
// viewer.setBackgroundColor (0.0, 0.0, 0.0);
   viewer_nor.addPointCloud (cloud);
   viewer_nor.addPointCloudNormals<PointType,pcl::Normal>(cloud, cloud_normals, 1, 0.05, "normals");
   viewer_nor.addCoordinateSystem (1.0);

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
    viewer_nor.spinOnce ();
  }


  return (0);
}
