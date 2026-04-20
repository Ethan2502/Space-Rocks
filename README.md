# Space Rocks - Gravity Simulation

An interactive gravitational N-body simulation with both web-based and C++ implementations.

## Overview

Space Rocks is a physics-based simulation project that models gravitational interactions between celestial bodies. The project features an interactive web interface ("Spacetime Sandbox") where you can create objects and observe them interact under gravitational forces, complemented by a high-performance C++ backend implementation. It includes forces of gravity and collision math.

## Features

### Web Interface (index.html)
- **Interactive Canvas**: Real-time visualization of gravitational interactions
- **Object Creation**: Click and drag to create new bodies with custom mass and velocity
- **Dynamic Physics**: Realistic gravity simulation using Newton's law of universal gravitation
- **Visual Feedback**: Color-coded bodies with procedurally generated textures
- **Responsive Design**: Adapts to window resizing

### C++ Backend (object.cpp)
- **Octree Data Structure**: Efficient spatial partitioning for N-body simulations
- **Optimized Calculations**: Reduces computational complexity from O(n²) to approximately O(n log n) Using Barnes-Hut algorithm
- **3D Vector Math**: Full 3D vector operations and physics calculations
- **Body System**: Manages mass, position, velocity, and acceleration for each object
