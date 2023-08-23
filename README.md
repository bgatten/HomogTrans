# Homogeneous Transformation Library
The Homogeneous Transformations Library (HTF) is a C++ library that provides a set of tools for working with homogeneous transformations, rotations, translations, and more. It is designed to be flexible and easy to integrate into your projects that involve robotics, computer graphics, or any application where you need to manipulate transformation matrices.

## Features

    Create and manipulate homogeneous transformation matrices.
    Perform rotations, translations, and combinations of transformations.
    Convert between rotation matrices, Euler angles, and quaternions.
    Decompose transformation matrices into translation and rotation components.

## Installation

    Clone the repository to your local machine:

```bash
git clone https://github.com/your-username/HomogeneousTransformations.git
``````
Navigate to the project directory:

```bash
cd HomogeneousTransformations
```

Create a build directory:

```bash
mkdir build
cd build
```

Generate the build files using CMake:

```bash
cmake ..
```
Build the library and example tests:

```bash
make
```

Run the example tests:

```bash
./tests
```
## Usage

To use the Homogeneous Transformations Library in your project, follow these steps:

    Include the necessary header in your source code:

```cpp
#include <htf/htf.h>
```

Create Htf objects and manipulate transformations as needed:

```cpp

    htf::Htf transform1;
    // Set transformations, rotations, translations, etc.
```
Build and link your project with the library and any required dependencies.

## Documentation

Detailed documentation for the library's classes and methods can be found in the Documentation directory.
License

This project is licensed under the MIT License - see the LICENSE file for details.
Contributing

Contributions are welcome! If you have suggestions, bug reports, or feature requests, please create an issue on the repository.
Acknowledgments

    This library is inspired by the need for efficient and reliable homogeneous transformations in robotics and computer graphics.