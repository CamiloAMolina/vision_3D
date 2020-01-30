#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <fstream>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>

int
  main (int argc, char** argv)
{
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
   //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
   pcl::io::loadPCDFile ("tuerca_scale_ascii.pcd", *cloud);
   
   // Create a set of indices to be used. For simplicity, we're going to be using the first 10% of the points in cloud
  //std::vector<int> indices (std::floor (cloud -> points.size () / 10));
  //std::cout<<(std::floor (cloud -> points.size () / 10))<<std::endl;
  //for (std::size_t i = 0; i < indices.size (); ++i) indices[i] = i;
   
// Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  //pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  
// Pass the indices
  //boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (indices));
  //ne.setIndices (indicesptr);
  
// Create an empty kdtree representation, and pass it to the normal estimation object.
// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  //pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
  ne.setSearchMethod (tree);
  
// Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  
// Use all neighbors in a sphere of radius 3cm
  //ne.setRadiusSearch (0.0001);
  ne.setKSearch (10);

// Compute the features
  ne.compute (*cloud_normals);

  // ofstream normals_file;
  // normals_file.open("normals_file.txt");
  // for (std::size_t j = 0;j < cloud_normals->points.size (); ++j){
  // //std::cout << cloud_normals->points[j].normal_x ;
  // //std::cout << cloud_normals->points[j].normal_y ;
  // //std::cout << cloud_normals->points[j].normal_z ;
  // //std::cout << cloud_normals->points[j].curvature << std::endl;
  // normals_file << cloud_normals->points[j].normal_x << ';';
  // normals_file << cloud_normals->points[j].normal_y << ';';
  // normals_file << cloud_normals->points[j].normal_z << ';';
  // normals_file << cloud_normals->points[j].curvature << '\n';
  // }
  // normals_file.close();
  
  pcl::PCDWriter writer;
  writer.write("cloud_normals.pcd", *cloud_normals, false);
  
    // Output has the PointNormal type in order to store the normals calculated by MLS
  //pcl::PointCloud<pcl::PointNormal> mls_points;

    // Reconstruct
  //ne.process (mls_points);

  // Save output
  //pcl::io::savePCDFile ("bun0-mls.pcd", cloud_normals);
    
    //pcl::PointCloud<pcl::PointNormal> cloud_c;
    //pcl::concatenateFields (cloud, cloud_normals, cloud_c);

//_____________________________________________________________________________
    pcl::visualization::PCLVisualizer viewer_nor ("Normals");
    viewer_nor.setBackgroundColor (0, 0, 0); //(255, 255, 255);
    viewer_nor.addPointCloud (cloud);
    viewer_nor.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals, 1, 0.05, "normals");
    //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "normals");
    viewer_nor.addCoordinateSystem (1.0);
   
   
   while (!viewer_nor.wasStopped ())
   {
viewer_nor.spinOnce();
//boost::this_thread::sleep (boost::posix_time::microseconds(100000));
   }

  return (0);
}
