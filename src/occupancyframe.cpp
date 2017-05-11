#include "occupancyframe.h"

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

OccupancyFrame::OccupancyFrame(const pangolin::Image<unsigned short> &depth)
	: depth_(depth)
	{}

bool OccupancyFrame::compute(Eigen::Matrix4f worldPose)
{
	for (size_t v=0; v<depth_.h; ++v) {
		for (size_t u=0; u<depth_.w; ++u) {
			unsigned short curDepth = depth_.RowPtr(v)[u];

			// Leave out the dead pixels
			if (curDepth == 0) continue;

			for (size_t rayZ=kStartZ; rayZ<kMaxZ; rayZ+=kStepZ) {
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
  		
//				std::cout << "rayZ = " << rayZ << std::endl;	
//				std::cout << "curDepth = " << curDepth << std::endl;
//				std::cout << "Current occupancy = " << curOccupancy << std::endl;
			}	
		}
	}
	
	return true;
}

// If generateOccupancyCloud is set to true, points behind objects will be added to the cloud
// to create a dense model
void OccupancyFrame::writePointCloud(std::string pointCloudFileName, Eigen::Matrix4f worldPose, bool generateOccupancyCloud)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	if (!generateOccupancyCloud) {
		size_t nPoints = depth_.w * depth_.h;
		cloud.width = nPoints;
		cloud.height = 1;
		cloud.is_dense = false;
		cloud.points.resize (cloud.width * cloud.height);
	}

	std::vector<double> modelX;
	std::vector<double> modelY;
	std::vector<double> modelZ;

	for (size_t v=0; v<depth_.h; ++v) {
		for (size_t u=0; u<depth_.w; ++u) {
			size_t depthIdx = v*depth_.h + u;

			// Downsample
			if (depthIdx % 2 != 0) continue;

			unsigned short curDepth = depth_.RowPtr(v)[u];

			// Don't bother processing dead pixels
			if (curDepth < 5) continue;

			if (!generateOccupancyCloud) {	
				double cloudZ = (double) curDepth / kCorrectionFactor;
				double cloudX = ((u - kOpticalCentreX)*cloudZ) / kFocalLengthX;
				double cloudY = ((v - kOpticalCentreY)*cloudZ) / kFocalLengthY;

				// Transform points to accurate world locations
				Eigen::RowVector4f curPoint;
				curPoint << cloudX, cloudY, cloudZ, 1;
				Eigen::MatrixXf transformedPoint = worldPose * curPoint.transpose();

				// Dehomogenise and add to point cloud
				cloud.points[depthIdx].x = transformedPoint(0,0) / transformedPoint(3,0);
				cloud.points[depthIdx].y = transformedPoint(1,0) / transformedPoint(3,0);
				cloud.points[depthIdx].z = transformedPoint(2,0) / transformedPoint(3,0);
			} else {
				for (size_t rayZ=curDepth; rayZ<kMaxZ; rayZ+=kStepZ) {
					double curZ = (double) rayZ / kCorrectionFactor;
					double curX = (u - kOpticalCentreX)*curZ / kFocalLengthX;
					double curY = (v - kOpticalCentreY)*curZ / kFocalLengthY;
				
					modelX.push_back(curX);
					modelY.push_back(curY);
					modelZ.push_back(curZ);	
				}
			}
		}
	}

	if (generateOccupancyCloud) {
		cloud.width = modelX.size();
		cloud.height = 1;
		cloud.is_dense = false;
		cloud.points.resize(cloud.width * cloud.height);

		for (size_t i=0; i<cloud.width; ++i) {
			cloud.points[i].x = modelX[i];
			cloud.points[i].y = modelY[i];
			cloud.points[i].z = modelZ[i];
		}
	}

	pcl::io::savePCDFileASCII (pointCloudFileName, cloud);
}

bool OccupancyFrame::writeToFile(std::string filename)
{
	return true;	
}
