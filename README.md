# Position-Based Dynamics 2D Solver

**Tom Favereau** and **Nicolas Pozo**

<p align="center">
  <img src="illustration.png" alt="Illustration" width="400">
</p>

## Overview

This project is a 2D physics solver built around Position-Based Dynamics (PBD), a method technique introduced by *Matthias Müller, Bruno Heidelberger, Marcus Hennix,* and *John Ratcliff* in their 2006 paper: “Position Based Dynamics.”  
PBD iteratively projects particle positions to satisfy geometric constraints and update the velocities afterward.

Our implementation uses:

- Qt for the rendering/UI layer
- Static constraints (planes, spheres, bowls) and spring clusters for compound objects
- UI interactions : particle spawning, emitters, cluster creation

### What I'm proud of

I optimized the simulation using a grid to resolve constraints and parallelization with QtConcurrent.

## Controls

- Hold **E** to spawn a small sphere at the center
- Press **C** to spawn a square cluster at the center
- Press **S** to spawn a soft body at the center
- Click the mouse to spawn a sphere at the mouse position


## Build & Run

Prerequisites:

- C++20 compiler.
- CMake ≥ 3.
- Qt 6 or 5.

```bash
git clone git@github.com:tom-favereau/solver_PBD.git
cd solver_PBD
cmake -S . -B build
cmake --build build --config Release
# Linux 
./build/SOLVER
# Max 
open ./build/SOLVEL.app