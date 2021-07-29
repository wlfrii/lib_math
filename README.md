# lib_math

This library is a mathematics extension library primarily designed for robot kinematics. And this library is based on Eigen.

## Requirement

  - <b>Linux / Mac OS</b> system platform.
  - <b>Eigen</b> library should be installed before <make&install> this library.

## How to use?

Download this library from github.
```bash
git clone https://github.com/wlfrii/lib_math.git
```

Before <make&install> this library, open the `lib_math/CMakeList.txt` file and find the command as follows, and then <b>comment these lines</b>.
```cmake
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX 
        ${REPO_ROOT}/buildtarget/ CACHE PATH "repo root" FORCE
    )
endif()
```

Since this library is primarily designed for myself, the library will be installed in a relative path as set in above lines to easy the management.  

After comment the command lines, do the <make&install> as follows.
```bash
cd lib_math
mkdir build
cd build
make -j
sudo make install
```

Then you can use this library in your project by adding the following commands in your `CMakeLists.txt`.
```cmake
find_package(lib_math REQUIRED)
...
target_link_libraries(${PROJECT_NAME} PUBLIC
    ${lib_math_LIBRARIES}
)
```

