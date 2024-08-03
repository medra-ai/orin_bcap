# Python bindings for Denso Controller Drivers "Orin BCAP"
Denso provided drivers in both Python and C for sending commands to robot,
but due to inability to control thread priority in Python because of the
global interpreter lock (GIL) we observe robot controller errors where
the trajectory buffer is not populated on time.
We built these python bindings around the C driver to overcome this issue.

C++ wrapper around Denso b-Cap C driver is based on https://github.com/medra-ai/denso-cpp

# Usage
0. Denso's BCAP C library assumes linux. To develop on Mac, use a Linux Docker image
Recommend setting up a VSCode Dev container with Ubuntu.
https://code.visualstudio.com/docs/devcontainers/tutorial

1. Install pybind and CMake
https://pybind11.readthedocs.io/en/stable/installing.html
https://cgold.readthedocs.io/en/latest/first-step/installation.html
```
$ pip install pybind11
$ apt install cmake
```

3. Build the medra_pybind python module
In the medra_pybind directory:
```
$ mkdir build
$ cd build
$ cmake ..
$ make
```
Python module in the build dir
`orin_bcap/medra_pybind/build/denso_pybind.cpython-310-aarch64-linux-gnu.so`
