# Cirobb

## 🚀 Overview

**Cirobb** is a small, readable 2D rigid body physics engine in C++, built around one state-of-the-art solver.

> 💡 For more complex simulations with multiple constraints and shapes, we recommend using **Box2D**, which is considered the best open-source 2D physics engine available.

## 📸 Captures

![Physics simulation with circles and rectangles](/images/img1.gif?raw=true)
![Advanced physics interactions](/images/img3.gif?raw=true)

## ⚡ Demos & Examples

- [WebAssembly Demo](https://joncodeofficial.github.io/cirobb/) (built and deployed from `main`)
- [YouTube Demo Video (v1.1.4)](https://youtu.be/j2p6qmOVA7M)

## Building

Desktop:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./bin/demo
```

Web (requires the [Emscripten SDK](https://emscripten.org/docs/getting_started/downloads.html)):

```bash
./build_web.sh
python3 -m http.server -d build-web/web
```

## 🔜 Future Features

<table>
  <tr>
    <td>⬜ Distance Constraint</td>
    <td>Optional feature for distance-based constraints</td>
  </tr>
  <tr>
    <td>⬜ Convex Polygons</td>
    <td>Collision Detection against convex polygons</td>
  </tr>
  <tr>
    <td>✅ Contact Points</td>
    <td><s>Change from Distance-Based to Feature-Based</s></td>
  </tr>
</table>

## 📚 Learning Resources

### Experts and Their Contributions

#### Erin Catto

- Iterative Dynamics with Temporal Coherence
- Modeling and Solving Constraints
- Soft Constraints
- Numerical Methods
- Continuous Collision Detection
- Understanding Constraints

#### Dirk Gregorius

- The Separating Axis test Between Convex Polyhedra
- Robust Contact Creation for physics Simulation

#### Erwin Coumans

- Forum Bullet
- Exploring MLCP Solvers And Featherstone

#### Tonge Richard

- Iterative Rigid Body Solvers 2012
- Iterative Rigid Body Solvers 2013

#### Brian Vincent Mirtich

- Impulse-based Dynamic Simulation of Rigid Body Systems

#### Kenny Erleben

- Book of Physics-Based Animation 2005
- Numerical Methods for Linear Complementarity Problems in Physics-based Animation

#### David Baraff

- Analytical Methods for Dynamic Simulation of Non-penetrating Rigid Bodies
- Fast Contact Force Computation for Nonpenetrating Rigid Bodies
- Linear-Time Dynamics using Lagrange Multipliers
- Physically Based Modeling, Pixar Animation Studios

#### Jim Van Verth

- Numerical Integration
- Understanding Rotations

#### Matthias Müller

- Position Based Dynamics

#### Michael B. Cline

- Post-Stabilization for Rigid Body Simulation with Contact and Constraints

#### Randy Gaul

- Separating Axis Test (SAT) and Support Points in 2D

### Books on Game Physics and Collision Detection

- Game Physics Pearls by Gino van den Bergen and Dirk Gregorius
- Real-Time Collision Detection by Christer Ericson

## 📝 Note

Cirobb is my personal project to practice physics for video games.
