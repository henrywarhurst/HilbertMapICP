cmake_minimum_required(VERSION 2.6.0)

project(ICP)

find_package(Pangolin REQUIRED)
find_package(CUDA REQUIRED)
find_package(Sophus REQUIRED)
find_package(PNG REQUIRED)

FIND_PATH(png++_INCLUDE_DIR
	NAMES  png++/color.hpp  png++/config.hpp png++/consumer.hpp png++/convert_color_space.hpp png++/end_info.hpp  png++/error.hpp  png++/ga_pixel.hpp png++/generator.hpp png++/gray_pixel.hpp png++/image.hpp png++/image_info.hpp png++/index_pixel.hpp png++/info.hpp png++/info_base.hpp  png++/io_base.hpp  png++/packed_pixel.hpp png++/palette.hpp png++/pixel_buffer.hpp  png++/pixel_traits.hpp png++/png.hpp png++/reader.hpp png++/require_color_space.hpp png++/rgb_pixel.hpp png++/rgba_pixel.hpp png++/streaming_base.hpp  png++/tRNS.hpp png++/types.hpp png++/writer.hpp)
  

#remove this as soon as eigen is shipped with FindEigen.cmake
  get_filename_component(EIGEN_ROOT "/usr/include/local/eigen3" PATH)
  if(PKG_CONFIG_FOUND)
    pkg_check_modules(PC_EIGEN eigen3)
  endif(PKG_CONFIG_FOUND)
  find_path(EIGEN_INCLUDE_DIRS Eigen/Core
    HINTS ${PC_EIGEN_INCLUDEDIR} ${PC_EIGEN_INCLUDE_DIRS} 
          "${EIGEN_ROOT}" "$ENV{EIGEN_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/Eigen 3.0.0" "$ENV{PROGRAMW6432}/Eigen 3.0.0"
          "$ENV{PROGRAMFILES}/Eigen" "$ENV{PROGRAMW6432}/Eigen"   
    PATH_SUFFIXES eigen3 include/eigen3 include)
  find_package_handle_standard_args(eigen DEFAULT_MSG EIGEN_INCLUDE_DIRS)
  set(EIGEN_DEFINITIONS ${EIGEN_DEFINITIONS} -DEIGEN_USE_NEW_STDVECTOR 
      -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
      
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIR})
include_directories(${PNG_INCLUDE_DIR})


file(GLOB srcs *.cpp)
file(GLOB cuda Cuda/*.cu)
file(GLOB containers Cuda/containers/*.cpp)

set(CUDA_ARCH_BIN "20 30 32 35 37 50 52 61" CACHE STRING "Specify 'real' GPU arch to build binaries for, BIN(PTX) format is supported. Example: 1.3 2.1(1.3) or 13 21(13)")
set(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")              

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})
include(CudaComputeTargetFlags.cmake)                  
APPEND_TARGET_ARCH_FLAGS()

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "-Xcompiler;-fPIC;-std=c++11")
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false") 

CUDA_COMPILE(cuda_objs ${cuda})

find_package(PCL 1.3 REQUIRED COMPONENTS common io)

include_directories(${PCL_INCLUDE_DIRS} ${png++_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_FLAGS "-O3 -msse2 -msse3 -Wall -std=c++11")

add_executable(ICP 
               ${srcs}
               ${cuda} 
               ${cuda_objs} 
               ${containers}
)

target_link_libraries(ICP
                      ${Pangolin_LIBRARIES}
                      ${Eigen_LIBRARIES}
                      ${CUDA_LIBRARIES}
					  ${PCL_COMMON_LIBRARIES}
					  ${PCL_IO_LIBRARIES}
					  ${PNG_LIBRARY}
)

