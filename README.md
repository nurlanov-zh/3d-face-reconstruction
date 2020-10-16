# 3D Face Reconstruction and Deformation Transfer using RGBD data
This project is the final project for the course [3D Scanning and Motion Capture](https://www.in.tum.de/cg/teaching/winter-term-2021/3d-scanning-motion-capture/) taught at TUM in SS20. For the detailed review please go to our [technical report](https://github.com/nurlanov-zh/event-based-odomety/blob/master/report.pdf)

# Install and build

Clone repo and submodules
```
git clone --recursive https://github.com/nurlanov-zh/3d-face-reconstruction.git
cd 3d-face-reconstruction
```

Install packages and build libs
```
./install_dependencies.sh
./build_submodules.sh
```
This may take a while, so be patient.

Make project
```
mkdir build
cd build
cmake ..
make -j2
```

# Project structure
Input data should be located in the folder `data/`. You need to download it before! Names are preserved the same as provided by tutors!

`src` contains the main code of the project

`src/common` for general data types

`src/utils` for dataset read, results write, visualize

The rest should be clear

Each folder cosists of `src` for `*.cpp` files and `include/<project_name>/` (see data reader as example) folder for `*.h` files (and probably `test` folder).

# How to start
There are several getters in `src/utils/data_reader/include/data_reader.h`. You can see them called in main.cpp. You can also call whatever you want from the data reader (PCA faces basis, expressions, albedo). After all your optimizations and refinements you should provide visualizer with your meshes (see main.cpp `setMesh` function call).

To run the app you need to stay in folder `build` and call `Build/bin/face_viewer`.
