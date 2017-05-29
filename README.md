# HilbertMapICP
Super fast implementation of ICP based upon [ICPCUDA](https://github.com/mp3guy/ICPCUDA). Theoretically this code runs on compute 2.0 devices and up, although all testing was completed on an __NVIDIA GTX 1070__. 

## Dependencies

* CUDA
* [Pangolin](https://github.com/stevenlovegrove/Pangolin)
* [Eigen](https://github.com/stevenlovegrove/eigen)
* [Sophus](https://github.com/stevenlovegrove/Sophus)
* [PCL](http://pointclouds.org/)

## Building

```bash
mkdir build
cd build
cmake ../src
make -j
```

## Running

You'll need to download one of the [TUM datasets](http://vision.in.tum.de/data/datasets/rgbd-dataset) to run:

```bash
./ICP ~/Desktop/rgbd_dataset_freiburg1_desk/ -v
```

## What to do with the output?

Copy the use the .occ files to incrementally build Hilbert Maps with [CudaHilbertMaps](https://github.com/henrywarhurst/CudaHilbertMaps2).
