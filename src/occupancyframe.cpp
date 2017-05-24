#include "occupancyframe.h"

#include <iostream>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

OccupancyFrame::OccupancyFrame(const pangolin::Image<unsigned short> &depth, 
							   const png::image< png::rgb_pixel > &rgb)
	: depth_(depth),
	  rgb_(rgb)
	{}

// 1/downsampleFactor is the percentage of pixels used
bool OccupancyFrame::compute(Eigen::Matrix4f worldPose, size_t downsampleFactor)
{
	for (size_t v=0; v<depth_.h; ++v) {
		for (size_t u=0; u<depth_.w; ++u) {
			unsigned short curDepth = depth_.RowPtr(v)[u];
			size_t depthIdx = v*depth_.h + u;

			// Leave out the dead pixels
			if (curDepth == 0) continue;

			// Downsample
			if (depthIdx % downsampleFactor != 0) continue;

			png::rgb_pixel pixel = rgb_.get_pixel(u,v);
			bool hasCrossedSurface = false;
			bool hasAddedSurfaceRgb = false;
			for (size_t rayZ=kStartZ; rayZ<kMaxZ; rayZ+=kStepZ) {
				double curZ = (double) rayZ / kCorrectionFactor;	
				double curX = (u - kOpticalCentreX)*curZ / kFocalLengthX;
				double curY = (v - kOpticalCentreY)*curZ / kFocalLengthY;

				int curOccupancy = -1;
				if (rayZ > (size_t) curDepth) {
					curOccupancy = 1;
					hasCrossedSurface = true;
				}

				if (hasCrossedSurface && !hasAddedSurfaceRgb) {
					hasAddedSurfaceRgb = true;
	       		    r_.push_back(pixel.red);
					g_.push_back(pixel.green);
					b_.push_back(pixel.blue);
				} else {
					// Push impossible values when we aren't at a surface
					r_.push_back(-1);
					g_.push_back(-1);
					b_.push_back(-1);
				}
				
				// Transform points to accurate world locations
				Eigen::RowVector4f curPoint;
				curPoint << curX, curY, curZ, 1;
				Eigen::Vector4f transformedPoint = worldPose * curPoint.transpose();

				x_.push_back(transformedPoint(0));
				y_.push_back(transformedPoint(1));
				z_.push_back(transformedPoint(2));
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
void OccupancyFrame::writePointCloud(	std::string pointCloudFileName, 
										Eigen::Matrix4f worldPose,
										size_t downsampleFactor,
										bool generateOccupancyCloud)
{
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
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
			if (depthIdx % downsampleFactor != 0) continue;

			unsigned short curDepth = depth_.RowPtr(v)[u];
			
			png::rgb_pixel pixel = rgb_.get_pixel(u,v);
			unsigned char redChannel = pixel.red;
			unsigned char greenChannel = pixel.green;
			unsigned char blueChannel = pixel.blue;
		
			// Don't bother processing dead pixels
			if (curDepth < 5) continue;

			if (!generateOccupancyCloud) {	
				double cloudZ = (double) curDepth / kCorrectionFactor;
				double cloudX = ((u - kOpticalCentreX)*cloudZ) / kFocalLengthX;
				double cloudY = ((v - kOpticalCentreY)*cloudZ) / kFocalLengthY;

				// Transform points to accurate world locations
				Eigen::RowVector4f curPoint;
				curPoint << cloudX, cloudY, cloudZ, 1;
				Eigen::Vector4f transformedPoint = worldPose * curPoint.transpose();

				// Dehomogenise and add to point cloud
				cloud.points[depthIdx].x = transformedPoint(0);
				cloud.points[depthIdx].y = transformedPoint(1);
				cloud.points[depthIdx].z = transformedPoint(2);

				// Add colour to the point cloud
				cloud.points[depthIdx].r = redChannel;
				cloud.points[depthIdx].g = greenChannel;
				cloud.points[depthIdx].b = blueChannel;
			
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

bool OccupancyFrame::writeToFile(std::string filename, std::string delim)
{
	std::ofstream occupancyFile (filename);
	if (!occupancyFile.is_open()) {
		std::cout << "There was a problem writing the file " << filename << std::endl;
		return false;
	}	

	if (x_.size() != y_.size() || y_.size() != z_.size() || z_.size() != occupancy_.size()) {
		std::cout << "File could not be written. Coordinate vectors are inconsistent sizes!" << std::endl;
		return false;
	}

	for (size_t i=0; i<x_.size(); ++i) {
		occupancyFile << x_[i] 			<< delim;
		occupancyFile << y_[i] 			<< delim;
		occupancyFile << z_[i] 			<< delim;
		occupancyFile << occupancy_[i] 	<< delim;
		occupancyFile << r_[i]			<< delim;
		occupancyFile << g_[i]			<< delim;
		occupancyFile << b_[i]			<< std::endl;	
	}
	occupancyFile.close();

	return true;	
}
