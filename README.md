# lib_localization
## Dependencies

This package requires of the following system libraries and packages

* [cmake](https://www.cmake.org "CMake's Homepage"), a cross-platform build system.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page), C++ template library for linear algebra

## Compilation and installation

Installation:

```shell
git clone https://github.com/AUROVA-LAB/lib_localization
cd lib_localization/
mkdir build
cd build
cmake .. 
make
sudo make install
```

## How to include it

To use this library in an other library or application, it is necessary add in the CMakeLists.txt file:

``` find_package(localization REQUIRED) ```

And Eigen's dependence is necessary:

``` 
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})
```

And link the libraries to the program

``` 
TARGET_LINK_LIBRARIES(<executable name> localization) 
```

