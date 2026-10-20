# Cirobb Roadmap

## Vision

Cirobb is my lab for studying the state of the art (SOTA) in rigid body simulation, built around one well-tuned solver.
It is not a Box2D clone: I take ideas from several engines (Box2D v3, Jolt, PhysX, Rapier...),
understand them in depth, measure them, and add my own variants.

Inspiration: Andrej Karpathy's small projects (nanoGPT, micrograd, llm.c). Little code
(target < 5K-10K lines), readable, with good practices and real optimizations.

**Goal:** not to compete with Box2D, Jolt, or commercial engines, but to understand the SOTA
in depth.

## Decisions

- **Language:** C++ (C++17/20), data-oriented style. No STL in the core, no exceptions or RTTI.
- **Code and docs in English:** identifiers, comments, and documentation.
- **Solver:** Erin Catto's soft step (TGS family + soft constraints + relax). No XPBD or AVBD.
- **Scope:** rigid body solver + contacts + one or two constraints (revolute, maybe distance/weld).
- **Hardware:** custom data structures (like Box2D v3), SoA, zero allocations in `Step`, graph coloring, SIMD (NEON first), optional multithreading.
- **Platforms:** portable core with no platform dependencies. Possible use from React Native (JSI/TurboModule) and WASM.
- **Not a single-file project:** a small engine split into a few well-separated files.

## Method: vibe learning

I write the engine; Claude acts as teacher and pair programmer. Each topic follows this loop:

1. **Concept**: the idea and the why, with the original source.
2. **Prediction**: before coding, answer what I think will happen.
3. **Implementation**: I write it. Hints come in levels: direction, then pseudocode, then code (only if I ask).
4. **Review**: code review, questions about the decisions, alternatives.
5. **Measurement**: compare against the previous version with numbers.
6. **Log**: record the idea, its origin, and the result in the [Idea log](#idea-log).

---

## Phases

### Phase 0: Understand what I already have

- [x] Move third-party dependencies into `vendor/`.
- [ ] (in progress) Lesson: walk through `Scene::Step` and understand the current loop (PGS + warm starting + Baumgarte/NGS).
  - Open question: which gives a more stable stack, 1 step with 8 iterations or 8 substeps with 1 iteration each? Why?
- [ ] Lesson: the Jacobian and effective mass of a contact (`Manifold::PreStep`).
- [ ] Lesson: warm starting and feature IDs (`Manifold::Update`).
- [ ] Known issues from the initial review, to understand and fix:
  - [ ] `std::map` keyed by pointers: solver order is not deterministic.
  - [ ] Memory leaks (`newShape()` without `delete`, `Scene::Clear`).
  - [ ] `min`/`max`/`absf` macros (double evaluation, missing parentheses, clash with `std`).
  - [ ] Frame-rate dependent damping (`pow(0.97f, damping)` without `dt`).
  - [ ] Unused `Contact::warmPoint`.
  - [ ] Unverified: possible false feature ID match in OBB-OBB when the reference box switches between A and B. **Build a scene that triggers it before assuming it happens.**

### Phase 1: Measure (no lab without measurements)

Benchmark approach: own headless harness, no new dependency (same idea as Box2D v3's `benchmark/`).
Physics benchmarks are whole-scene runs with physics metrics, which generic micro-benchmark
libraries do not cover. Micro-benchmarks and profilers come later, only when needed.

- [ ] Split the core (`cirobb/`) from the demo; the core does not depend on OpenGL/GLFW.
- [ ] Headless benchmark executable (`bench/`):
  - [ ] Fixed `dt`, fixed number of steps, warmup steps before timing.
  - [ ] Timing with `std::chrono::steady_clock`; several runs, report median and min.
  - [ ] Always measure Release builds (`-O2`/`-O3`), never Debug.
  - [ ] Output CSV and keep results in `bench/results/` to compare over time.
- [ ] Fixed scenes:
  - [ ] Tall box pyramid.
  - [ ] Heavy box on a light box (mass ratio).
  - [ ] Chain / pendulum (once joints exist).
  - [ ] Fast bodies against thin walls (tunneling).
- [ ] Metrics: ms per step, max penetration, drift / energy, bodies before it blows up, determinism (state hash).
- [ ] Measure the current PGS as the **baseline**.
- [ ] Later, if needed:
  - [ ] Micro-benchmarks of isolated functions (collision routines, SIMD math): nanobench (single header) or Google Benchmark via CMake `FetchContent`.
  - [ ] Profiling: Instruments (macOS), `perf` (Linux), Tracy (instrumentation).

### Phase 2: Paradigm: data-oriented design (DOD)

One of the first things to do. Each step comes with a lesson on the idea behind it and why
it beats the current OOP design (cache, memory ownership, determinism, preparing for SIMD).
Not ECS: physics engines like Box2D v3 and Jolt are DOD internally, not ECS.

- [ ] Lesson: what DOD is and why (Mike Acton, CppCon 2014; Richard Fabian, *Data-Oriented Design*).
- [ ] Exercise: list which `RigidBody` fields `WarmStarting` and `ApplyImpulse` read and write; compare with `sizeof(RigidBody)`.
- [ ] Measure the baseline before changing anything (Phase 1).
- [ ] The world owns memory: `CreateBody` / `DestroyBody` returning IDs (index + generation, like Jolt's `BodyID`). No user `new`.
- [ ] Shapes as data: `enum ShapeType` + one struct per type, no virtuals (keep the dispatch table).
- [ ] Contiguous contacts: replace `std::map` with an array + pair hash set keyed by `(idA, idB)`. Gives determinism too.
- [ ] Split hot and cold state: compact `SolverBody` with only what the solver touches.
- [ ] Precompute per-contact data once (anchors `ra`/`rb`, effective masses) instead of every iteration.
- [ ] `Step` as stages over arrays: broadphase, narrowphase, prepare, warm start, solve, integrate, store impulses.
- [ ] Measure again and log the result.

### Phase 3: Soft step

- [ ] Substepping (N substeps of `dt/N`).
- [ ] TGS-style relative separation: `separation + dot(n, dp_B - dp_A)` without rerunning collision.
- [ ] Soft constraints: contact as a spring (frequency in Hz, damping ratio zeta) to derive impulse coefficients.
- [ ] Relax iterations (pass without bias).
- [ ] Restitution in a separate pass using the pre-solve relative velocity.
- [ ] Speculative contacts.
- [ ] Compare against the baseline (Phase 1) and the DOD version (Phase 2).

### Phase 4: Shapes and contacts

- [ ] Convex polygon (the box becomes a polygon).
- [ ] SAT + clipping (or GJK) for polygon-polygon and polygon-circle.
- [ ] Manifold with robust feature IDs.
- [ ] (Optional) Capsules.

### Phase 5: Data and memory

- [ ] Custom containers (dynamic array, ID pool, pair hash set, bitset).
- [ ] Arena / stack allocator: zero allocations inside `Step`.
- [ ] Memory layout tuned for the solver: compact body state, contiguous and aligned arrays (prepares SIMD in Phase 8).
- [ ] Determinism (stable ordering, floating point flags such as `-ffp-contract=off`).

### Phase 6: Broadphase

- [ ] Dynamic AABB tree (or sort-and-sweep) instead of O(n^2).
- [ ] Measure with many-body scenes.

### Phase 7: Joints

- [ ] Revolute joint (soft).
- [ ] (Optional) Distance or weld joint.

### Phase 8: Exploit the hardware

- [ ] Constraint graph coloring (no write conflicts between contacts of the same color).
- [ ] Custom SIMD layer: NEON + SSE, with a scalar fallback.
- [ ] SIMD solver (4/8 contacts at a time).
- [ ] Optional multithreading (own task system or user-provided).

### Phase 9: Scale

- [ ] Islands.
- [ ] Sleeping.

### Phase 10: Platforms

- [ ] WASM (update the web demo).
- [ ] React Native: C++ TurboModule / JSI, transforms in a shared buffer (`ArrayBuffer`) read once per frame, rendering with react-native-skia.

### Phase 11: My own variant

- [ ] Own ideas that come up in earlier phases (record them in the log as they appear).

---

## Idea log

| Date | Idea | Origin | Measured result | Decision |
|---|---|---|---|---|
| | | | | |

## References

- Erin Catto: Box2D blog, "Solver2D" (2024), comparison of PGS, TGS, soft step, XPBD...
- Erin Catto: GDC talks "Iterative Dynamics with Temporal Coherence", "Modeling and Solving Constraints", "Soft Constraints", "Understanding Constraints".
- Box2D v3 source (custom data structures, graph coloring, SIMD, solver sets).
- Macklin et al.: "Small Steps in Physics Simulation" (why substepping works).
- PhysX: TGS solver documentation.
- Jolt Physics, Rapier: architecture and performance decisions.
- Dirk Gregorius: SAT and "Robust Contact Creation for Physics Simulation".
- Christer Ericson: *Real-Time Collision Detection*.
