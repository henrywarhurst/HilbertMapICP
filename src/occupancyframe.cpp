#include "occupancyframe.h"

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

OccupancyFrame::OccupancyFrame(const unsigned char *depth, size_t width, size_t height)
	: depth_(depth),
	  width_(width),
	  height_(height)
	{}

bool OccupancyFrame::compute()
{
	const size_t maxZ 		= 1000;
	const size_t startZ 	= 1;
	const size_t stepZ 		= 100;

	// TODO: Delete PCL stuff once we verify the clouds are correct
	pcl::PointCloud<pcl::PointXYZ> cloud;
	size_t nPoints = width_*height_;
	std::cout << std::endl;
	std::cout << "nPoints = " << nPoints << std::endl;
	cloud.width = nPoints;
	cloud.height = 1;
	cloud.is_dense = true;
	cloud.points.resize (cloud.width * cloud.height);
	std::cout << "Number of points in the cloud is " << cloud.points.size() << std::endl;

	for (size_t v=0; v<height_; ++v) {
		for (size_t u=0; u<width_; ++u) {
			size_t depthIdx = v*height_ + u;
			double curDepth = (double) depth_[depthIdx];
			// TODO: Delete this, it's just to check the pointcloud
			// is generated properly ******************************
			double tstZ = curDepth / kCorrectionFactor;
			double tstX = (u - kOpticalCentreX)*tstZ / kFocalLengthX;
			double tstY = (v - kOpticalCentreY)*tstZ / kFocalLengthY;
			// Send to pcl and save as pcd file to view
			cloud.points[depthIdx].x = tstX;
			cloud.points[depthIdx].y = tstY;
			cloud.points[depthIdx].z = tstZ;

		
			// ****************************************************

			for (size_t rayZ=startZ; rayZ<maxZ; rayZ+=stepZ) {
				double curZ = (double) rayZ / kCorrectionFactor;	
				double curX = (u - kOpticalCentreX)*curZ / kFocalLengthX;
				double curY = (v - kOpticalCentreY)*curZ / kFocalLengthY;

				int curOccupancy = -1;
				if ((double) rayZ > curDepth) {
					curOccupancy = 1;
				}
				
				x_.push_back(curX);
				y_.push_back(curY);
				z_.push_back(curZ);
				occupancy_.push_back(curOccupancy);
			
//				std::cout << "rayZ = " << rayZ << std::endl;	
//				std::cout << "curDepth = " << curDepth << std::endl;
//				std::cout << "Current occupancy = " << curOccupancy << std::endl;
			}	
		}
	}
	
	// TODO: Delete PCL stuff
	//pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
	
	return true;
}

bool OccupancyFrame::writeToFile(std::string filename)
{
	return true;	
}
