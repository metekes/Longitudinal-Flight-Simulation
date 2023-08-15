# Longitudinal Flight Simulation



## Introduction

This simulator is designed to replicate the longitudinal flight of a fixed-wing airplane, allowing you to control its altitude and velocity. The simulation generates logs, and those logs are stored in the output file. The content of the logs is:

   - Time [s]
   - Velocity [m/s]
   - Reference Velocity [m/s]
   - Altitude [m]
   - Reference Altitude [m]
   - Pitch Angle [deg]
   - Angle of Attack [deg]

## Installation
Follow these steps to compile and run your project:

1. **Download the Project:** Clone this repository and navigate to the project directory:

    ```bash
    git clone https://github.com/metekes/Longitudinal-Flight-Simulation.git
    cd Longitudinal-Flight-Simulation
    ```

2. **CMake and Build:** Run CMake to generate build files, then use `make` to compile the project:

    ```bash
    mkdir build && cd build
    cmake .. && make
    ```

3. **Run the Project:** Execute the compiled binary:

    ```bash
    ./main
    ```
4. **Move logs:** Move the output file to the project directory:

    ```bash
    mv data.txt ../
    ```

### Prerequisites

Before you begin, please ensure you have the [Eigen library](http://eigen.tuxfamily.org) installed on your system.

