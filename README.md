# lib_math

This library is a mathematics extension library based on [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). Some useful tiny functions that were created in some of my projects are rearranged in this project to facilitate my work.

The library will be updated continuously.

## Requirement

  - <b>CMake</b> tool (version should be larger than __3.10__). 
  - <b>Eigen</b> library (version should be no less that __3.4.0__) .

## How to use?

Download this library from github.
```bash
git clone https://github.com/wlfrii/lib_math.git
```

### I. Configure the library in your project 

#### I.1 For Linux / Mac OS (recommended)

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

#### I.2 For Windows

Open your terminal and check your cmake supported Visual Studio version first by type the command
```bash
cmake -G
```

Then find the generator name that has an __asterisk *__ in front of the name, which is the support generator. 
For example, `* Visual Studio 16 2019`.

After unpackaged the repository, do the <make&install> in your terminal as follows.
```bash
cd lib_math
mkdir build
cd build
cmake "Visual Studio 16 2019" ..
make -j
sudo make install
```

Finally, open the __lib_math.sln__ in your `build/` folder by your Visual Studio and compile the solution. After compile done, you can include this library into you project by add the path of headers `lib_math/export/` and static library `lib_math.lib`.

__Possible problem__:
  + Cannot find Eigen3. 
  __Fix__: *Comment the Line:44-Line:49 of the CMakeLists.txt, and replace `${EIGEN3_INCLUDE_DIRS}` in Line:58 of the CMakeLists.txt by your source code path of Eigen3.*


### II. Use the library

The library could then be used by only including the following header file in your source file. Note, all the interfaces are defined in `mmath{}` namespace. A tiny example is given as follows.

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

Some other usage that could be the reference can be found in [lib_math/test](https://github.com/wlfrii/lib_math/tree/main/test/src) folder.


__NOTE__: The `precision` for calculation of kinemaices related functions is change to `float`, which is generally enough for most of application. To enable `double` precision, define `LIB_MATH_KINE_DOUBLE` before include this library, as follows.
```c++
#define LIB_MATH_KINE_DOUBLE
#include <lib_math/lib_math>
```


## Interfaces Manual

Download this repository and see the interfaces manual by opening `./doc/lib_math_doc.html` via your browser.
