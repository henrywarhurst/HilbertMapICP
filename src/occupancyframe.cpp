#include "occupancyframe.h"

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

OccupancyFrame::OccupancyFrame(const pangolin::Image<unsigned short> &depth)
	: depth_(depth)
	{}

bool OccupancyFrame::compute()
{
	const size_t maxZ 		= 10000;
	const size_t startZ 	= 1;
	const size_t stepZ 		= 100;

	for (size_t v=0; v<depth_.h; ++v) {
		for (size_t u=0; u<depth_.w; ++u) {

			unsigned short curDepth = depth_.RowPtr(v)[u];
			// Leave out the dead pixels
			if (curDepth == 0) continue;

			for (size_t rayZ=startZ; rayZ<maxZ; rayZ+=stepZ) {
				double curZ = (double) rayZ / kCorrectionFactor;	
				double curX = (u - kOpticalCentreX)*curZ / kFocalLengthX;
				double curY = (v - kOpticalCentreY)*curZ / kFocalLengthY;

				int curOccupancy = -1;
				if (rayZ > (size_t) curDepth) {
					curOccupancy = 1;
				}
				
				x_.push_back(curX);
				y_.push_back(curY);
				z_.push_back(curZ);
				occupancy_.push_back(curOccupancy);
  		
				std::cout << "rayZ = " << rayZ << std::endl;	
				std::cout << "curDepth = " << curDepth << std::endl;
				std::cout << "Current occupancy = " << curOccupancy << std::endl;
			}	
		}
	}
	
	return true;
}

void OccupancyFrame::writePointCloud(std::string pointCloudFileName)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	size_t nPoints = depth_.w * depth_.h;
	cloud.width = nPoints;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize (cloud.width * cloud.height);

	for (size_t v=0; v<depth_.h; ++v) {
		for (size_t u=0; u<depth_.w; ++u) {
			size_t depthIdx = v*depth_.h + u;

			unsigned short curDepth = depth_.RowPtr(v)[u];

			double cloudZ = (double) curDepth / kCorrectionFactor;
			double cloudX = ((u - kOpticalCentreX)*cloudZ) / kFocalLengthX;
			double cloudY = ((v - kOpticalCentreY)*cloudZ) / kFocalLengthY;
			// Send to pcl and save as pcd file to view
			cloud.points[depthIdx].x = cloudX;
			cloud.points[depthIdx].y = cloudY;
			cloud.points[depthIdx].z = cloudZ;
		}
	}

	pcl::io::savePCDFileASCII (pointCloudFileName, cloud);
}

bool OccupancyFrame::writeToFile(std::string filename)
{
	return true;	
}
