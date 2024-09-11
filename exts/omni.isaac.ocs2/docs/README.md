# OCS2 Robot Controllers: Motion planners using MPC-SLQ

This extension provides software for MPC-based motion planners for various robots. It uses Pinocchio for rigid body
dynamics and HPP-FCL for collision checking.

The code has been tested on Ubuntu 18.04LTS and Ubuntu 20.04LTS.

Currently the optimal control formulation for the following systems are supported:

* Mobile Manipulator: https://leggedrobotics.github.io/ocs2/robotic_examples.html#mobile-manipulator
* Fixed-arm Manipulator: https://leggedrobotics.github.io/ocs2/robotic_examples.html#franka-panda

## Citation

If you use this work in an academic context, please cite the following publication:

> Mittal, Mayank, David Hoeller, Farbod Farshidian, Marco Hutter, and Animesh Garg.
> **"Articulated object interaction in unknown scenes with whole-body mobile manipulation."**
> 2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (2022).

```text
@article{mittal2021articulated,
  title={Articulated object interaction in unknown scenes with whole-body mobile manipulation},
  author={Mittal, Mayank and Hoeller, David and Farshidian, Farbod and Hutter, Marco and Garg, Animesh},
  journal={arXiv preprint arXiv:2103.10534},
  year={2021}
}
```

> Farshidian, Farbod, Maryam Kamgarpour, Diego Pardo, and Jonas Buchli. **"Sequential
> linear quadratic optimal control for nonlinear switched systems."** IFAC-PapersOnLine
> 50, no. 1 (2017): 1463-1469.

```text
@article{farshidian2017sequential,
  title={Sequential linear quadratic optimal control for nonlinear switched systems},
  author={Farshidian, Farbod and Kamgarpour, Maryam and Pardo, Diego and Buchli, Jonas},
  journal={IFAC-PapersOnLine},
  volume={50},
  number={1},
  pages={1463--1469},
  year={2017},
  publisher={Elsevier}
}
```
