# Space Rocks - Gravity Simulation

An interactive gravitational N-body simulation with both web-based and C++ implementations.

## Overview

Space Rocks is a physics-based simulation project that models gravitational interactions between celestial bodies. The project features an interactive web interface ("Spacetime Sandbox") where you can create objects and observe them interact under gravitational forces, complemented by a high-performance C++ backend implementation.

## Features

### Web Interface (index.html)
- **Interactive Canvas**: Real-time visualization of gravitational interactions
- **Object Creation**: Click and drag to create new bodies with custom mass and velocity
- **Dynamic Physics**: Realistic gravity simulation using Newton's law of universal gravitation
- **Visual Feedback**: Color-coded bodies with procedurally generated textures
- **Responsive Design**: Adapts to window resizing

### C++ Backend (object.cpp)
- **Octree Data Structure**: Efficient spatial partitioning for N-body simulations
- **Optimized Calculations**: Reduces computational complexity from O(n²) to approximately O(n log n)
- **3D Vector Math**: Full 3D vector operations and physics calculations
- **Body System**: Manages mass, position, velocity, and acceleration for each object

## Usage

### Running the Web Simulation

1. Open `index.html` in a web browser
2. **Create objects**: Click and drag on the canvas to create new bodies
   - Drag distance determines initial velocity
   - Size scales with mass
3. **Observe interactions**: Watch as gravity pulls bodies toward each other
4. **Reset**: Refresh the page to clear the simulation

### Building the C++ Backend

```bash
clang++ -O2 object.cpp -o simulate_space
```

## Technical Details

### Physics Model
- **Gravity Constant**: G = 5.0 (web) / 50.0 (C++)
- **Time Step**: DT = 0.05s (web) / 0.01s (C++)
- **Force Calculation**: F = G × m₁ × m₂ / r²

### Simulation Parameters
- **JavaScript**: Client-side calculations with 2D projection
- **C++**: 3D simulation with octree optimization for large N

## Files

- `index.html` - Web-based interactive simulation interface
- `object.cpp` - C++ implementation with N-body physics and octree acceleration
- `simulate_space` - Compiled executable for the C++ backend
- `object.o` - Compiled object file

## Future Enhancements

- [ ] Collision detection and merging
- [ ] Energy conservation analysis
- [ ] Performance metrics display
- [ ] Save/load simulation states
- [ ] 3D visualization option
- [ ] Network multiplayer support

## License

[Add your license here]

---

**Created**: Space Rocks Gravity Simulation Project
