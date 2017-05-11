#ifndef OCCUPANCY_FRAME_H_
#define OCCUPANCY_FRAME_H_

#include <string>
#include <vector>

#include <pangolin/image/image_io.h>

class OccupancyFrame
{
	public:

		OccupancyFrame(pangolin::Image<unsigned short> &depth, size_t width=640, size_t height=480);

		bool compute();

		bool writeToFile(std::string filename);	

	private:	
		pangolin::Image<unsigned short> depth_;

		size_t width_;
		size_t height_;

		std::vector<double> x_;
		std::vector<double> y_;
		std::vector<double> z_;
		
		std::vector<int> occupancy_;

		static constexpr const double kCorrectionFactor = 5000.0 	;
		static constexpr const double kFocalLengthX 	= 525.0		;
		static constexpr const double kFocalLengthY 	= 525.0		;
		static constexpr const double kOpticalCentreX 	= 319.5		;
		static constexpr const double kOpticalCentreY 	= 239.5		;
};

#endif /* OCCUPANCY_FRAME_H_ */
