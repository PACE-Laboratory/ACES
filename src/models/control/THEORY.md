# Background Theory for the Control Implementation

## Assumptions
- The control law is causal

## Inputs and Outputs

**Inputs:**
- Controller state, $\bm{x}_\mathrm{ctrl} \in \mathbb{R}^{n_\mathrm{ctrl}}$
- Rigid body state, $\bm{x}_\mathrm{rb}$
- Actuator state, $\bm{\delta} \in \mathbb{R}^{p_\mathrm{act}}$
- Generalized wind velocity, $\bm{\varpi} \in \mathbb{R}^6$
- Extended state estimate, $\hat{\bm{x}} \in \mathbb{R}^{p_\mathrm{est}}$
- Time, $t$

**Outputs:**
- Control input, $\bm{u} \in \mathbb{R}^{p_\mathrm{ctrl}}$
- Controller state time derivative $\dot{\bm{x}}_\mathrm{ctrl} \in \mathbb{R}^{n_\mathrm{ctrl}}$

## General control law form
The controller has the dynamics
$$
    \dot{\bm{x}}_\mathrm{ctrl} = \bm{f}_\mathrm{ctrl}(\bm{x}_\mathrm{ctrl},\bm{x}_\mathrm{rb},\bm{\delta},\bm{w},\hat{\bm{x}},t)
$$
where $\bm{x}_\mathrm{ctrl} \in \mathbb{R}^{n_\mathrm{ctrl}}$ is the state of the controller. The control input (the output of the contoller) is
$$
    \bm{u} = \bm{h}_\mathrm{ctrl}(\bm{x}_\mathrm{ctrl},\bm{x}_\mathrm{rb},\bm{\delta},\bm{w},\hat{\bm{x}},t)
$$

### Common Control Law Architectures
- Linear static state feedback
$$
    \delta\bm{u} = -\bm{K} \delta\bm{x}
$$
where $\delta\bm{u} = \bm{u} - \bm{u}_\mathrm{ref}$ and $\delta\bm{x} = \bm{x} - \bm{x}_\mathrm{ref}$.
- Linear dynamic output feedback
$$
    \dot{\bm{x}}_\mathrm{ctrl} = \bm{A}_\mathrm{ctrl} \bm{x}_\mathrm{ctrl} + \bm{B}_\mathrm{ctrl} \delta\bm{y}
$$
$$
    \delta\bm{u} = \bm{C}_\mathrm{ctrl} \bm{x}_\mathrm{ctrl} + \bm{D}_\mathrm{ctrl} \delta\bm{y}
$$
where $\delta\bm{y} = \bm{y} - \bm{y}_\mathrm{ref}$ such that
$$
    \bm{y} = \bm{h}(\bm{x}) \quad\text{and}\quad \bm{y}_\mathrm{ref} = \bm{h}(\bm{x}_\mathrm{ref})
$$
- Nonlinear static state feedback
$$
    \bm{u} = \bm{h}_\mathrm{ctrl}(\bm{x})
$$
- Nonlinear dynamic output feedback (most general)
$$
    \dot{\bm{x}}_\mathrm{ctrl} = \bm{f}_\mathrm{ctrl}(\bm{x}_\mathrm{ctrl},\bm{x},t)
$$
$$
    \bm{u} = \bm{h}_\mathrm{ctrl}(\bm{x}_\mathrm{ctrl},\bm{x},t)
$$