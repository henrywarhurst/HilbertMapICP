#include "ICPOdometry.h"
#include "occupancyframe.h"

#include <iomanip>
#include <fstream>
#include <chrono>
#include <pangolin/image/image_io.h>

std::ifstream asFile;
std::ifstream colourFile;
std::string directory;

void tokenize(const std::string & str, std::vector<std::string> & tokens, std::string delimiters = " ")
{
    tokens.clear();

    std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
    std::string::size_type pos = str.find_first_of(delimiters, lastPos);

    while (std::string::npos != pos || std::string::npos != lastPos)
    {
        tokens.push_back(str.substr(lastPos, pos - lastPos));
        lastPos = str.find_first_not_of(delimiters, pos);
        pos = str.find_first_of(delimiters, lastPos);
    }
}

void loadImage(pangolin::Image<unsigned char> &rgb)
{   
    std::string currentLine;  
    std::vector<std::string> tokens;
    std::vector<std::string> timeTokens;

    do
    {   
        getline(colourFile, currentLine);
        tokenize(currentLine, tokens);
    } while(tokens.size() > 2);

    std::string imageLoc = directory;
    imageLoc.append(tokens[1]);

    pangolin::TypedImage imageRaw = pangolin::LoadImage(imageLoc, pangolin::ImageFileTypePng);

	pangolin::Image<unsigned char> imageOutput((unsigned char *) imageRaw.ptr, 
												imageRaw.w, 
												imageRaw.h, 
												3 * imageRaw.w * sizeof(unsigned char));

	for (size_t i=0; i<imageRaw.h; ++i) {
		for (size_t j=0; j<imageRaw.w*3; ++j) {
			rgb.RowPtr(i)[j] = imageRaw.RowPtr(i)[j];	
		}
	}

	std::cout << "first pixel " << (unsigned short) rgb.RowPtr(0)[0] << std::endl;
	std::cout << "second pixel " << (unsigned short) rgb.RowPtr(0)[1] << std::endl;
	std::cout << "third pixel " << (unsigned short) rgb.RowPtr(0)[2] << std::endl;

	std::cout << "Width = " << imageRaw.w << " Height = " << imageRaw.h << std::endl;
}


uint64_t loadDepth(pangolin::Image<unsigned short> & depth)
{
    std::string currentLine;
    std::vector<std::string> tokens;
    std::vector<std::string> timeTokens;

    do
    {
        getline(asFile, currentLine);
        tokenize(currentLine, tokens);
    } while(tokens.size() > 2);

    if(tokens.size() == 0)
        return 0;

    std::string depthLoc = directory;
    depthLoc.append(tokens[1]);

    pangolin::TypedImage depthRaw = pangolin::LoadImage(depthLoc, pangolin::ImageFileTypePng);

    pangolin::Image<unsigned short> depthRaw16((unsigned short*)depthRaw.ptr, depthRaw.w, depthRaw.h, depthRaw.w * sizeof(unsigned short));

    tokenize(tokens[0], timeTokens, ".");

    std::string timeString = timeTokens[0];
    timeString.append(timeTokens[1]);

    uint64_t time;
    std::istringstream(timeString) >> time;

    for(unsigned int i = 0; i < 480; i++)
    {
        for(unsigned int j = 0; j < 640; j++)
        {
            depth.RowPtr(i)[j] = depthRaw16(j, i)  / 5;
        }
    }

    depthRaw.Dealloc();

    return time;
}

void outputFreiburg(const std::string filename, const uint64_t & timestamp, const Eigen::Matrix4f & currentPose)
{
    std::ofstream file;
    file.open(filename.c_str(), std::fstream::app);

    std::stringstream strs;

    strs << std::setprecision(6) << std::fixed << (double)timestamp / 1000000.0 << " ";

    Eigen::Vector3f trans = currentPose.topRightCorner(3, 1);
    Eigen::Matrix3f rot = currentPose.topLeftCorner(3, 3);

    file << strs.str() << trans(0) << " " << trans(1) << " " << trans(2) << " ";

    Eigen::Quaternionf currentCameraRotation(rot);

    file << currentCameraRotation.x() << " " << currentCameraRotation.y() << " " << currentCameraRotation.z() << " " << currentCameraRotation.w() << "\n";

    file.close();
}

uint64_t getCurrTime()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
}

int main(int argc, char * argv[])
{
    assert((argc == 2 || argc == 3) && "Please supply the depth.txt dir as the first argument");

    directory.append(argv[1]);

    if(directory.at(directory.size() - 1) != '/')
    {
        directory.append("/");
    }

    std::string associationFile = directory;
    associationFile.append("depth.txt");

    asFile.open(associationFile.c_str());

	std::string imageAssociationFile = directory;
	imageAssociationFile.append("rgb.txt");

	colourFile.open(imageAssociationFile.c_str());	

    pangolin::ManagedImage<unsigned short> firstData(640, 480);
    pangolin::ManagedImage<unsigned short> secondData(640, 480);

	pangolin::ManagedImage<unsigned char> rgbData(640, 480);
	
    pangolin::Image<unsigned short> firstRaw(firstData.w, firstData.h, firstData.pitch, (unsigned short*)firstData.ptr);
    pangolin::Image<unsigned short> secondRaw(secondData.w, secondData.h, secondData.pitch, (unsigned short*)secondData.ptr);

	pangolin::Image<unsigned char> rgbRaw(rgbData.w, rgbData.h, rgbData.pitch, (unsigned char *)rgbData.ptr);

    ICPOdometry icpOdom(640, 480, 319.5, 239.5, 528, 528);

    assert(!asFile.eof() && asFile.is_open());

    loadDepth(firstRaw);
    uint64_t timestamp = loadDepth(secondRaw);

    Sophus::SE3d T_wc_prev;
    Sophus::SE3d T_wc_curr;

    std::ofstream file;
    file.open("output.poses", std::fstream::out);
    file.close();

    cudaDeviceProp prop;

    cudaGetDeviceProperties(&prop, 0);

    std::string dev(prop.name);

    std::cout << dev << std::endl;

    float mean = std::numeric_limits<float>::max();
    int count = 0;

    int threads = 224;
    int blocks = 96;

    int bestThreads = threads;
    int bestBlocks = blocks;
    float best = mean;

    if(argc == 3)
    {
        std::string searchArg(argv[2]);

        if(searchArg.compare("-v") == 0)
        {
            std::cout << "Searching for the best thread/block configuration for your GPU..." << std::endl;
            std::cout << "Best: " << bestThreads << " threads, " << bestBlocks << " blocks (" << best << "ms)"; std::cout.flush();

            float counter = 0;

            for(threads = 16; threads <= 512; threads += 16)
            {
                for(blocks = 16; blocks <= 512; blocks += 16)
                {
                    mean = 0.0f;
                    count = 0;

                    for(int i = 0; i < 5; i++)
                    {
                        icpOdom.initICPModel(firstRaw.ptr);
                        icpOdom.initICP(secondRaw.ptr);

                        uint64_t tick = getCurrTime();

                        T_wc_prev = T_wc_curr;

                        Sophus::SE3d T_prev_curr = T_wc_prev.inverse() * T_wc_curr;

                        icpOdom.getIncrementalTransformation(T_prev_curr, threads, blocks);

                        T_wc_curr = T_wc_prev * T_prev_curr;

                        uint64_t tock = getCurrTime();

                        mean = (float(count) * mean + (tock - tick) / 1000.0f) / float(count + 1);
                        count++;
                    }

                    counter++;

                    if(mean < best)
                    {
                        best = mean;
                        bestThreads = threads;
                        bestBlocks = blocks;
                    }

                    std::cout << "\rBest: " << bestThreads << " threads, " << bestBlocks << " blocks (" << best << "ms), " << int((counter / 1024.f) * 100.f) << "%    "; std::cout.flush();
                }
            }

            std::cout << std::endl;
        }
    }

    threads = bestThreads;
    blocks = bestBlocks;

    mean = 0.0f;
    count = 0;

    T_wc_prev = Sophus::SE3d();
    T_wc_curr = Sophus::SE3d();

    while(!asFile.eof())
    {
        icpOdom.initICPModel(firstRaw.ptr);
        icpOdom.initICP(secondRaw.ptr);

        uint64_t tick = getCurrTime();

        T_wc_prev = T_wc_curr;

        Sophus::SE3d T_prev_curr = T_wc_prev.inverse() * T_wc_curr;

        icpOdom.getIncrementalTransformation(T_prev_curr, threads, blocks);

        T_wc_curr = T_wc_prev * T_prev_curr;

        uint64_t tock = getCurrTime();

        mean = (float(count) * mean + (tock - tick) / 1000.0f) / float(count + 1);
        count++;

        std::cout << std::setprecision(4) << std::fixed
                  << "\rICP: "
                  << mean << "ms";
                  std::cout.flush();

        std::swap(firstRaw, secondRaw);

		Eigen::Matrix4f worldPose = T_wc_curr.cast<float>().matrix();
		std::cout << std::endl;
		std::cout << "World pose is: " << std::endl;
		std::cout << worldPose << std::endl;
        outputFreiburg("output.poses", timestamp, worldPose);

        timestamp = loadDepth(secondRaw);

		// Get RGB data
		loadImage(rgbRaw);

		std::cout << "Got past the image load" << std::endl;
 		// OccupancyFrames ---------------------------------------------------------
		size_t downsampleFactor = 1;
		OccupancyFrame curOccupancyFrame(secondRaw, rgbRaw);
		bool isOccupancyComputationOk = curOccupancyFrame.compute(worldPose, downsampleFactor);
		curOccupancyFrame.writePointCloud(std::to_string(count) + ".pcd", worldPose, downsampleFactor, false);
		bool isOccupancyWriteOk = curOccupancyFrame.writeToFile(std::to_string(count) + ".occ", ",");
   	}

    std::cout << std::endl;

    std::cout << "ICP speed: " << int(1000.f / mean) << "Hz" << std::endl;

    return 0;
}

