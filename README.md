# Install and build

Clone repo and submodule
```
git clone --recursive https://gitlab.lrz.de/00000000014A4E08/3d-face-reconstruction.git
cd 3d-face-reconstruction
```

Install packages and build libs
```
./install_dependencies.sh
./build_submodules.sh
```

Make project
```
mkdir build
cd build
cmake ..
make -j2
```

# Project stucture
Input data should be located in folder `data/`.

`src` contains main code of the project

`src/common` for general data types

`src/utils` for dataset read, results write, visualize

The rest should be clear

Each folder cosists of `src` for `*.cpp` files and `include/<project_name>/` folder for `*.h` files (and probably `test` folder).
