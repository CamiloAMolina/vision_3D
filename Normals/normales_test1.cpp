#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>

int
  main (int argc, char** argv)
{
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::io::loadPCDFile ("tuerca_scale_ascii.pcd", *cloud);
   
// Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  
// Create an empty kdtree representation, and pass it to the normal estimation object.
// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  
// Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  
// Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.0001);

// Compute the features
  ne.compute (*cloud_normals);

//_____________________________________________________________________________
   pcl::visualization::PCLVisualizer viewer("PCL Viewer");
   //viewer.setBackgroundColor (0.0, 0.0, 0.0);
   viewer.addPointCloud (cloud);
   viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals);
   viewer.addCoordinateSystem (1.0);
   
   
   while (!viewer.wasStopped ())
   {
viewer.spinOnce();
//boost::this_thread::sleep (boost::posix_time::microseconds(100000));
   }

  return (0);
}
