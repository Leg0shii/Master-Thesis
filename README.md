# Safety-Critical Path Planning with LQR-CBF-RRT* in 2D Dynamic Environments

Master's Thesis, M.Sc. in Applied Artificial Intelligence, IU University of Applied Sciences  
Benjamin Müller · Supervisor: Dr. Usman Akhtar · December 2025

---

## What This Is

This thesis takes the LQR-CBF-RRT\* motion planning framework ([Yang et al., 2025](https://arxiv.org/abs/2304.00790)), which only works with static obstacles, and extends it to handle **dynamic environments** with moving circular obstacles. The planner combines RRT\* sampling, LQR steering, and a **time-varying Control Barrier Function (CBF)** that predicts obstacle positions and rejects unsafe edges during tree expansion and rewiring.

The evaluation answers three questions:

1. **Safety:** Does the planner keep collision rates below 1% over 3,000 roll-outs? Yes. Zero collisions observed; Wilson 95% upper bound at 0.13%.
2. **Overhead:** What does the CBF layer cost? Median planning time goes up ~8.6x while path length stays about the same.
3. **Sensitivity:** How do the CBF gain α, nominal speed v_nom, and extension length step_len affect results? 1,799 out of 1,800 successes across an 18-cell parameter grid. Clearance is mostly driven by speed and extension length, not CBF gain.

## What I Built (vs. the Base Framework)

The starting point is the [open-source LQR-CBF-RRT\* implementation](https://github.com/mingyucai/LQR_CBF_rrtStar) by Yang et al., which only handles static obstacles. I added:

- Time-varying CBF safety filter that predicts obstacle motion along candidate edges using constant-velocity kinematics
- Time-stamped tree expansion where nodes carry arrival times that propagate through rewiring
- Dynamic obstacle model with configurable count, speed, and radius ranges
- Experiment harness for batch execution of 3,000+ roll-outs with structured logging and CSV export
- Replay and audit tooling: JSON artifacts for collision cases, CLI-based replay, animation export, per-case diagnostics
- Paired evaluation protocol with seed-matched baseline comparison and Wilson confidence intervals

All of this is on the [`dyn-tvcbf`](https://github.com/Leg0shii/LQR_CBF_rrtStar/tree/dyn-tvcbf) branch of my fork (123 commits).

## Repo Structure

The code lives in my fork. This repo has the thesis document and links to the implementation:

| Resource | Link |
|----------|------|
| **Thesis PDF** | [`master_thesis_lqr_cbf_rrt.pdf`](./master_thesis_lqr_cbf_rrt.pdf) |
| **Implementation** | [Leg0shii/LQR_CBF_rrtStar (branch `dyn-tvcbf`)](https://github.com/Leg0shii/LQR_CBF_rrtStar/tree/dyn-tvcbf) |
| **Original framework** | [mingyucai/LQR_CBF_rrtStar](https://github.com/mingyucai/LQR_CBF_rrtStar) |
| **Base paper** | [Yang et al., 2023, arXiv:2304.00790](https://arxiv.org/abs/2304.00790) |

### Key Files (on `dyn-tvcbf` branch)

```
├── LQR_CBF_rrtStar_linear.py   # Planner core: RRT* expansion, rewiring, tree structure
├── LQR_planning.py              # Discrete LQR steering with optional CBF safety layer
├── CBFsteer.py                  # CBF-QP with dynamic obstacle prediction
├── experiments.py               # Batch experiment driver (3000+ roll-outs)
├── replay_experiment.py         # Collision replay and animation tool
├── plotting.py                  # Visualization utilities
├── env.py                       # Environment and obstacle configuration
├── linear_dynamic_model/        # Linear system experiments
├── nonlinear_dynamic_model/     # Nonlinear system experiments
├── results/                     # Experiment logs and CSV summaries
└── prisma_tables/               # PRISMA literature review records
```

## Results

| Metric | Value |
|--------|-------|
| Collision rate (3,000 roll-outs, v_max = 1.0 m/s) | **0.00%** (Wilson 95% upper: 0.13%) |
| Median planning time overhead | **~8.6x** vs. LQR-RRT\* baseline |
| Median path length difference | **~1.7 m** (not significant) |
| Sensitivity grid success rate | **1,799 / 1,800** (99.94%) |

## Tech Stack

Python 3.10 · NumPy · SciPy · Gurobi (QP solver) · Matplotlib

## Citation

```
@mastersthesis{mueller2025safety,
  title   = {Safety-Critical Path Planning with LQR-CBF-RRT* in 2D Dynamic Environments},
  author  = {M{\"u}ller, Benjamin},
  school  = {IU University of Applied Sciences},
  year    = {2025},
  month   = {December},
  type    = {Master's Thesis}
}
```

Extends the work of:

```
@inproceedings{yang2025lqrcbfrrt,
  title     = {LQR-CBF-RRT*: Safe and Optimal Motion Planning},
  author    = {Yang, Guang and Cai, Mingyu and Ahmad, Ahmad and Prorok, Amanda and Tron, Roberto and Belta, Calin},
  booktitle = {2025 American Control Conference (ACC)},
  pages     = {3700--3705},
  year      = {2025}
}
```
