#include "occupancyframe.h"

OccupancyFrame::OccupancyFrame(const unsigned char *depth, size_t width, size_t height)
	: depth_(depth),
	  width_(width),
	  height_(height)
	{}

bool OccupancyFrame::compute()
{
	size_t maxZ 	= 50000;
	size_t startZ 	= 10;
	size_t stepZ 	= 1000;

	for (size_t v=0; v<height_; ++v) {
		for (size_t u=0; u<width_; ++u) {
			size_t depthIdx = v*height_ + u*width_;
			double curDepth = (double) depth_[depthIdx];

			for (size_t rayZ=startZ; rayZ<maxZ; rayZ+=stepZ) {
				double curZ = (double) rayZ / kCorrectionFactor;	
				double curX = (u - kOpticalCentreX)*curZ / kFocalCentreX;
				double curY = (v - kOpticalCentreY)*curZ / kFocalCentreY;

				int curOccupancy = -1;
				if (rayZ > curDepth) {
					curOccupancy = 1;
				}
				
				x_.push_back(curX);
				y_.push_back(curY);
				z_.push_back(curZ);
				occupancy_.push_back(curOccupancy);
			}	
		}
	}

}

bool OccupancyFrame::writeToFile(std::string filename)
{
	return true;	
}
