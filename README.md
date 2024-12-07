# Tracking Filters Implementation
This repository contains implementations of linear and nonlinear tracking filters, including:

## Kalman Filters:
- Linear Kalman Filter (LKF)
- Extended Kalman Filter (EKF)
- Unscented Kalman Filter (UKF)
- Particle Filter (PF)

## Extended Target Tracking Filters:
- Random Matrix-based methods

## Target Tracking Under Clutter:
- Bernoulli Filters
- Gaussian Sum Filters (GSF)

# Purpose
The primary goal of this project is to provide a clear understanding of the mathematical foundations of tracking filters. Unlike existing Python libraries that often abstract away the underlying operations, this repository focuses on implementations built solely using matrix operations from scratch.

This approach is ideal for:

- Researchers and students who want to grasp the mathematical details of target tracking.
- Developers seeking block-based, easily modifiable code instead of relying on complex multiple black-box libraries.

## Features
- **Mathematical Focus**:
  Demonstrates the full mathematical workflow for various filters.
  Simplifies understanding by avoiding unnecessary abstractions.

- **Simulation and Testing**:
Includes test scripts for simulation scenarios.
Provides methods for generating simulation data and modeling sensor characteristics.

- **Modular Code**:
Each filter is implemented in a clear, modular way, making it easy to modify and adapt.

## Applications
This repository is suitable for:
- Understanding and implementing tracking filters in target tracking systems.
- Simulating cluttered environments and evaluating tracking performance.
- Learning how to model sensors and generate simulation data for tracking problems.

## Contributions
Contributions are welcome! If you'd like to contribute, please fork the repository and submit a pull request. For major changes, open an issue first to discuss what you would like to change.

## License
This project is licensed under the MIT License. See the LICENSE file for details.
