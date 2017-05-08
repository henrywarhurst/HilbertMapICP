#ifndef OCCUPANCY_FRAME_H_
#define OCCUPANCY_FRAME_H_

#include <string>
#include <vector>

class OccupancyFrame
{
	public:

		OccupancyFrame(const unsigned char *depth, size_t width=640, size_t height=480);

		bool compute();

		bool writeToFile(std::string filename);	

	private:	
		const unsigned char *depth_;

		size_t width_;
		size_t height_;

		std::vector<double> x;
		std::vector<double> y;
		std::vector<double> z;
		
		std::vector<int> occupancy;

		static constexpr const double kFocalCentreX 	= 525.0;
		static constexpr const double kFocalCentreY 	= 525.0;
		static constexpr const double kOpticalCentreX 	= 319.5;
		static constexpr const double kOpticalCentreY 	= 239.5;
};

#endif /* OCCUPANCY_FRAME_H_ */
