#include "occupancyframe.h"

#include <iostream>

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

	for (size_t v=0; v<height_; ++v) {
		for (size_t u=0; u<width_; ++u) {
			size_t depthIdx = v*height_ + u*width_;
			double curDepth = (double) depth_[depthIdx];

			// TODO: Delete this, it's just to check the pointcloud
			// is generated properly ******************************
			double tstZ = curDepth / kCorrectionFactor;
			double tstX = (u - kOpticalCentreX)*tstZ / kFocalLengthX;
			double tstY = (v - kOpticalCentreY)*tstZ / kFocalLengthY;
			// Send to pcl and save as pcd file to view
			
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
			
				std::cout << "rayZ = " << rayZ << std::endl;	
				std::cout << "curDepth = " << curDepth << std::endl;
				std::cout << "Current occupancy = " << curOccupancy << std::endl;
			}	
		}
	}
	
	return true;
}

bool OccupancyFrame::writeToFile(std::string filename)
{
	return true;	
}
