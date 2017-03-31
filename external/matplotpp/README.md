# matplotpp
**MATPLOT++** is a Matlab like simple plotting framework in C++. In cases where you need to make plots of data generated in C/C++, `matplotpp` enables you to generate 2D/3D plots, histograms, scatterplots, etc, just by adding a few lines of code, without any external plotting tool such as MATLAB or Gnuplot.

![logo](http://wiki.matplotpp.googlecode.com/git/logo.png)

MATPLOT++ features are the following:
* MATLAB like command set.
* Cross platform compatibility on Linux, MacOS, and Windows. `matplotpp` is based on **OpenGL/GLUT** which is a simple cross-platform windowing **API**.
* High quality vector output: PS, EPS, PDF, and SVG.

# Note
This repository is a fork of the [matplotpp](https://code.google.com/p/matplotpp/) library by **Yuichi Katori**. The repository is not maintained. It is merely a modified version of the project that uses `cmake`.

# Dependencies
To install the required dependencies on Linux/Ubuntu, run
```bash
sudo apt-get install freeglut3-dev libglew-dev
```

# Compilation

```bash
git clone https://github.com/pAIgn10/matplotpp.git
cd matplotpp

mkdir build
cd build

# to configure and compile
cmake ..
make

# to run the examples (e.g.)
./ex101_creating_a_plot

# to install the library
sudo make install
```
