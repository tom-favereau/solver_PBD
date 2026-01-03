# Position-Based Dynamics 2D Solver

**Tom Favereau** and **Nicolas Pozo**

[demo](illustration.png)

## Overview

This project is a 2D physics solver built around Position-Based Dynamics (PBD), a method technique introduced by *Matthias Müller, Bruno Heidelberger, Marcus Hennix,* and *John Ratcliff* in their 2006 paper: “Position Based Dynamics.”  
PBD iteratively projects particle positions to satisfy geometric constraints and update the velocities afterward.

Our implementation uses:

- Qt for the rendering/UI layer.
- A grid with mutex locking for parallel contact resolution using QtConcurrent.
- Static constraints (planes, spheres, bowls) and spring clusters for compound objects.
- UI interactions : particle spawning, emitters, cluster creation.


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
# On Linux/macOS:
./build/bin/td9_pbd
# On Windows (PowerShell):
...?