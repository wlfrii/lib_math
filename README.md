# lib_math

This library is a mathematics extension library based on [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). Some useful tiny functions that were created in some of my projects are rearranged in this project to facilate my work.

The library will be updated continuously.

## Requirement

  - <b>Linux / Mac OS</b> system platform.
  - <b>Eigen</b> library should be installed before <make&install> this library.

## How to use?

Download this library from github.
```bash
git clone https://github.com/wlfrii/lib_math.git
```

After unpackaged the repository, do the <make&install> in your terminal as follows.
```bash
cd lib_math
mkdir build
cd build
cmake ..
make -j
sudo make install
```

Then you can use this library in your project by adding the following commands in your `CMakeLists.txt`.
```cmake
# Find lib_math
find_package(lib_math REQUIRED)

# Inlcude headers path of lib_math to your project
target_include_directories(${PROJECT_NAME}  PUBLIC
    $<BUILD_INTERFACE:${lib_math_INCLUDE_DIRS}>
)

# Link lib_math to your project
target_link_libraries(${PROJECT_NAME} PUBLIC
    ${lib_math_LIBRARIES}
)
```

The library could then be used by only including the following header file in your source file. Note, all the interfaces are defined in `mmath{}` namespace. A tiny examples is given as follows.

```c++
#include <lib_math/lib_math>
#include <vector>

int main()
{
    // Your 2D coordinates
    std::vector<Eigen::Vector2f> pts = {...}; 
    // Fit GaussianCurve
    mmath::GaussianCurve gauss = mmath::fitGuassianCurve(pts);
    // Show results
    printf("Gaussian paramters: a:%f, mu:%f, sigma:%f\n",
        gauss.a, gauss.mu, gauss.sigma);

    return 0;
}
```

## Interfaces Manual

Download this repository and see the interfaces manual by opening `./doc/lib_math_doc.html` by your browser.
