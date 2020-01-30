#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
 //...
 int
 main ()
 {
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
   //... populate cloud
   pcl::io::loadPCDFile ("cloud_normals.pcd", *cloud);

   pcl::visualization::PCLVisualizer axes; 
   axes.addCoordinateSystem (1.0, 0);
   axes.addPointCloud (cloud);
   
   while (!axes.wasStopped ())
   {
axes.spinOnce(100);
boost::this_thread::sleep (boost::posix_time::microseconds(100000));
   }
return 0;
 }
