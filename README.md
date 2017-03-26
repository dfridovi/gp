# Gaussian Processes

[![Build Status](https://travis-ci.org/dfridovi/gp.svg?branch=master)](https://travis-ci.org/dfridovi/gp)
[![License](https://img.shields.io/badge/license-BSD-blue.svg)](https://github.com/dfridovi/gp/blob/master/LICENSE)

A homebrewed C++ library for Gaussian processes. **gp** is developed by [David Fridovich-Keil](http://people.eecs.berkeley.edu/~dfk/), a second-year PhD student in the Berkeley [Hybrid Systems Lab](http://hybrid.eecs.berkeley.edu) and the [Berkeley Artificial Intelligence Research (BAIR) Lab](http://bair.berkeley.edu).

## Status
**gp** is still under active development. I hope to have a first release soon though, so stay tuned!

## Structure
All source code is located in `src/`; headers are in `include/`; unit tests are in `/test/`; and executables are in `exec/`. Compiled binaries will be placed in `bin/`.

## Dependencies
I may miss a few here, but here is a list of dependencies:

* [Eigen](http://eigen.tuxfamily.org/dox/) (header-only linear algebra library)
* Gflags (Google's command-line flag manager)
* Glog (Google's logging tool)

All of these may be installed very easily. If you run into any trouble, though, I am more than happy to help you figure out what's going on. Just post an [issue](https://github.com/dfridovi/gp/issues) on this repository and I will reply as soon as possible.

## Usage
You'll need to begin by building the repository. From the top directory, type the following sequence of commands:

```
mkdir bin
mkdir build
cd build
cmake ..
make -j4
```

This should build all tests and executables. In order to run tests, you can run the following command:

```
./run_tests
```

from within the `build/` directory you just made. All the tests should pass, and none should take more than a second or so to run.

Executables are automatically placed within the `bin/` directory that you created. To run them, just type `./(name-of-executable)`.

To the extent that it makes sense, all parameters are accessible from the command line via Gflags. For help with command line options, simply run the following command:

```
./(name-of-executable) --help
```

## API documentation
I've been using Doxygen to auto-generate web-based [documentation](https://dfridovi.github.io/gp/documentation/html/). Although I do not follow the Doxygen guidelines for writing comments, auto-generation still seems to do a fairly reasonable job.
