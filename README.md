# Double Pendulum Simulator 🎭

[![MATLAB](https://img.shields.io/badge/MATLAB-0076A8?style=for-the-badge&logo=mathworks&logoColor=white)](https://www.mathworks.com/products/matlab.html)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg?style=for-the-badge)](https://opensource.org/licenses/MIT)

A comprehensive MATLAB simulation and visualization of chaotic double pendulum dynamics. This project demonstrates the fascinating world of deterministic chaos through numerical integration and interactive visualization.

<p align="center">
  <i>Experience the beauty of chaos theory in classical mechanics</i>
</p>

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Mathematical Background](#mathematical-background)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Examples](#examples)
- [Theory](#theory)
- [Contributing](#contributing)
- [Author](#author)
- [License](#license)

## 🎯 Overview

The double pendulum is one of the simplest dynamical systems that exhibits **chaotic behavior**. This project provides:

- **Numerical Simulation**: Solves the equations of motion using ODE45
- **Real-time Animation**: Visualizes pendulum motion and trajectory traces
- **Energy Analysis**: Tracks kinetic, potential, and total energy
- **Interactive App**: MATLAB App Designer GUI for easy parameter exploration
- **Phase Space Plots**: Visualizes the system's chaotic attractors

### What is a Double Pendulum?

A double pendulum consists of two pendulums attached end to end. While simple in construction, it demonstrates:
- **Sensitive Dependence on Initial Conditions**: Tiny changes lead to drastically different outcomes
- **Deterministic Chaos**: Predictable equations produce unpredictable behavior
- **Energy Conservation**: Despite chaotic motion, total energy remains constant
- **Complex Dynamics**: Rich mathematical structure with multiple equilibria

## ✨ Features

- 🎨 **Real-time Animation** with trajectory traces
- 📊 **Energy Visualization** showing conservation over time
- 🔬 **Phase Space Analysis** revealing chaotic attractors
- 🎛️ **Interactive GUI** (App Designer) for parameter tuning
- 📈 **Comprehensive Plots** of angles, velocities, and energies
- ⚡ **Efficient Computation** using MATLAB's ODE45 solver
- 📝 **Well-Documented Code** with detailed comments
- 🔧 **Customizable Parameters** (masses, lengths, initial conditions)

## 🧮 Mathematical Background

### Equations of Motion

The double pendulum is governed by the Lagrangian:

```
L = T - V
```

Where:
- **T** = Kinetic Energy
- **V** = Potential Energy

Using the Euler-Lagrange equations:

```
d/dt(∂L/∂θ̇ᵢ) - ∂L/∂θᵢ = 0
```

We derive the angular accelerations:

```matlab
α₁ = f₁(θ₁, θ₂, ω₁, ω₂, m₁, m₂, l₁, l₂, g)
α₂ = f₂(θ₁, θ₂, ω₁, ω₂, m₁, m₂, l₁, l₂, g)
```

### Energy Equations

**Kinetic Energy:**
```
KE₁ = ½m₁l₁²ω₁²
KE₂ = ½m₂[l₁²ω₁² + l₂²ω₂² + 2l₁l₂ω₁ω₂cos(θ₁-θ₂)]
```

**Potential Energy:**
```
PE₁ = m₁gy₁
PE₂ = m₂gy₂
```

**Total Energy:**
```
E = KE₁ + KE₂ + PE₁ + PE₂ = constant
```

## 📥 Installation

### Prerequisites

- **MATLAB** (R2019b or later recommended)
- **No additional toolboxes required** (uses built-in functions only)

### Quick Start

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/soumik-saha/double-pendulum-simulator.git
   cd double-pendulum-simulator
   ```

2. **Open MATLAB**:
   ```matlab
   cd('double-pendulum-simulator/matlab')
   ```

3. **Run Simulation**:
   ```matlab
   double_pendulum_simulation
   ```

4. **Or Use Interactive App**:
   ```matlab
   Double_pendulum_simulator  % Opens App Designer GUI
   ```

## 🚀 Usage

### Option 1: Command-Line Simulation

```matlab
% Navigate to matlab folder
cd matlab/

% Run the simulation
double_pendulum_simulation

% The script will:
% 1. Solve equations of motion
% 2. Animate the pendulum
% 3. Show energy evolution
% 4. Generate summary plots
```

### Option 2: Interactive App

```matlab
% Open the App Designer application
Double_pendulum_simulator

% Use the GUI to:
% - Adjust masses (m₁, m₂)
% - Modify lengths (l₁, l₂)
% - Set initial angles (θ₁, θ₂)
% - Control simulation speed
% - Start/Stop/Reset simulation
```

### Customizing Parameters

Edit the simulation script to change parameters:

```matlab
%% System Parameters
m1 = 2;      % Mass of first pendulum (kg)
m2 = 1;      % Mass of second pendulum (kg)
l1 = 1;      % Length of first rod (m)
l2 = 2;      % Length of second rod (m)
g = 9.8;     % Gravitational acceleration (m/s^2)

%% Initial Conditions
theta1_0 = 2.2;     % Initial angle 1 (rad)
theta2_0 = 1.6;     % Initial angle 2 (rad)
omega1_0 = 0;       % Initial angular velocity 1 (rad/s)
omega2_0 = 0;       % Initial angular velocity 2 (rad/s)
```

## 📁 Project Structure

```
double-pendulum-simulator/
├── matlab/
│   ├── double_pendulum_simulation.m      # Main simulation script
│   ├── odefunction.m                      # ODE solver function
│   └── Double_pendulum_simulator.mlapp    # Interactive App Designer GUI
├── README.md                              # This file
├── LICENSE                                # MIT License
└── .gitignore                             # Git ignore rules
```

## 🎬 Examples

### Example 1: Standard Configuration

```matlab
% Equal masses, equal lengths
m1 = 1; m2 = 1;
l1 = 1; l2 = 1;
theta1_0 = pi/2; theta2_0 = pi/2;
```
**Result**: Symmetric oscillations with moderate chaos

### Example 2: Chaotic Configuration

```matlab
% Slightly different initial conditions
theta1_0 = 2.2;
theta2_0 = 1.6;
```
**Result**: Highly chaotic motion with complex trajectories

### Example 3: Energy Comparison

```matlab
% Different mass ratios
m1 = 2; m2 = 1;  % vs.  m1 = 1; m2 = 2;
```
**Result**: Observe different energy distributions between pendulums

## 📚 Theory

### Why is it Chaotic?

1. **Nonlinear Coupling**: The two pendulums are nonlinearly coupled through the equations of motion
2. **Sensitive Dependence**: Initial conditions differing by 0.001° can lead to completely different trajectories after ~10 seconds
3. **Unpredictability**: Long-term behavior cannot be predicted despite deterministic equations
4. **Strange Attractors**: Phase space reveals fractal-like structures

### Applications

- **Weather Prediction**: Similar chaotic systems
- **Solar System Dynamics**: Orbital mechanics exhibit chaos
- **Robotics**: Multi-link manipulator control
- **Biomechanics**: Human limb dynamics
- **Chaos Theory Education**: Classic demonstration system

### Key Observations

1. **Energy Conservation**: Total energy remains constant (numerical errors < 0.1%)
2. **Periodic vs Chaotic**: Small angles → periodic motion; Large angles → chaos
3. **Lyapunov Exponents**: Positive for chaotic regime
4. **Poincaré Sections**: Reveal hidden structure in chaos

## 🛠️ Advanced Usage

### Analyzing Sensitivity

```matlab
% Compare two nearly identical initial conditions
theta1_0_a = 2.2;
theta1_0_b = 2.201;  % Only 0.001 rad difference

% Run both and plot difference
% Observe exponential divergence
```

### Poincaré Section

```matlab
% Sample when θ₁ = 0 and ω₁ > 0
% Plot (θ₂, ω₂) to reveal structure
```

### Energy Error Analysis

```matlab
% Check energy conservation
energy_error = max(abs(E_total - E_total(1)));
fprintf('Energy error: %.6f J\n', energy_error);
```

## 🐛 Troubleshooting

### Common Issues

**Animation is too fast/slow**
```matlab
% Adjust pause time in animation loop
pause(0.01)  % Decrease for faster, increase for slower
```

**Out of memory error**
```matlab
% Reduce simulation time or increase time step
t_end = 50;   % Instead of 100
dt = 0.05;    % Instead of 0.03
```

**Plot doesn't show**
```matlab
% Make sure figure windows aren't closed
% Check MATLAB graphics settings
```

## 🤝 Contributing

Contributions welcome! Areas for improvement:

- [ ] Add Runge-Kutta integrator comparison
- [ ] Implement Lyapunov exponent calculation
- [ ] Create 3D visualization
- [ ] Add damping and forcing terms
- [ ] Export animation to video
- [ ] Add more predefined scenarios
- [ ] Optimize performance for longer simulations

## 👨‍💻 Author

**Soumik Saha**
- Institution: Bangladesh University of Engineering and Technology (BUET)
- Department: Electrical and Electronic Engineering
- GitHub: [@soumik-saha](https://github.com/soumik-saha)

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- Classical mechanics principles from Goldstein's "Classical Mechanics"
- Numerical methods from MATLAB documentation
- Chaos theory concepts from Strogatz's "Nonlinear Dynamics and Chaos"
- BUET Faculty for guidance and support

## 📖 References

1. Strogatz, S. H. (2015). *Nonlinear Dynamics and Chaos*. Westview Press.
2. Goldstein, H., Poole, C., & Safko, J. (2002). *Classical Mechanics* (3rd ed.). Addison Wesley.
3. Shinbrot, T., et al. (1992). "Chaos in a double pendulum." *American Journal of Physics*, 60(6), 491-499.

## 📞 Contact

For questions or collaboration:
- Email: 2006011@eee.buet.ac.bd
- GitHub Issues: [Create an issue](https://github.com/soumik-saha/double-pendulum-simulator/issues)

---

<p align="center">
  <b>⭐ Star this repository if you find it interesting!</b>
</p>

<p align="center">
  Explore the fascinating world of chaos 🌪️
</p>
