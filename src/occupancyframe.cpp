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
	size_t stepZ 	= 500;

	for (size_t v=0; i<height; ++i) {
		for (size_t u=0; j<width; ++j) {
			size_t depthIdx = i*height + j*width;
			for (size_t rayZ=startZ; rayZ<maxZ; rayZ+=stepZ) {
				double curZ = (double) depth_[depthIdx] / kCorrectionFactor;	
				double curX = (u - kOpticalCentreX)*Z / kFocalCentreX;
				double curY = (v - kOpticalCentreY)*Z / kFocalCentreY;
			}	
		}
	}

}

bool OccupancyFrame::writeToFile(std::string filename)
{
	return true;	
}
