#ifndef OCCUPANCY_FRAME_H_
#define OCCUPANCY_FRAME_H_

#include <string>
#include <vector>

#include <pangolin/image/image_io.h>

#include <eigen3/Eigen/Dense>

class OccupancyFrame
{
	public:

		OccupancyFrame(const pangolin::Image<unsigned short> &depth,
					   const pangolin::Image<unsigned char> &rgb);

		bool compute(Eigen::Matrix4f worldPose, size_t downsampleFactor);

		void writePointCloud(	std::string pointCloudFileName, 
							 	Eigen::Matrix4f worldPose, 
								size_t downsampleFactor, 
								bool generateOccupancyCloud);

		bool writeToFile(std::string filename, std::string delim);	

	private:	
		const pangolin::Image<unsigned short> depth_;
		const pangolin::Image<unsigned char> rgbMap_;

		std::vector<double> x_;
		std::vector<double> y_;
		std::vector<double> z_;
		
		std::vector<int> occupancy_;

		static constexpr const double kCorrectionFactor = 1000.0 	;
		static constexpr const double kFocalLengthX 	= 525.0		;
		static constexpr const double kFocalLengthY 	= 525.0		;
		static constexpr const double kOpticalCentreX 	= 319.5		;
		static constexpr const double kOpticalCentreY 	= 239.5		;

		static constexpr const size_t kMaxZ				= 2000		;
		static constexpr const size_t kStepZ			= 50		;
		static constexpr const size_t kStartZ			= 100		;
};

#endif /* OCCUPANCY_FRAME_H_ */
