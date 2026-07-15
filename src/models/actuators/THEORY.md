# Theory: Actuator Model

Symbols and Julia identifiers follow the [canonical notation](@ref notation).

## Assumptions
- The actuator model is deterministic and only depends on the air-relative flow conditon.
- There is no pure delay; however, continuous-time delay approximations are supported.

## Inputs and Outputs

**Inputs:**
- Air-relative velocity, $\bm{v}_r \in \mathbb{R}^3$
- Apparent air-relative angular velocity, $\bm{\omega}_r \in \mathbb{R}^3$
- Actuator input, $\bm{u} \in \mathbb{R}^{p_\mathrm{act}}$
- Actuator model state, $\bm{x}_\mathrm{act} \in \mathbb{R}^{n_\mathrm{act}}$

**Outputs:**
- Physical actuator output or configuration, $\bm{\delta} \in \mathbb{R}^{p_\mathrm{act}}$
- Actuator model state time derivative, $\dot{\bm{x}}_\mathrm{act} \in \mathbb{R}^{n_\mathrm{act}}$

## Functional dependece of the actuator model
The actuator model state $\bm{x}_\mathrm{act} \in \mathbb{R}^{n_\mathrm{act}}$ satisfies
```math
	\dot{\bm{x}}_\mathrm{act} = \bm{f}_\mathrm{act}(\bm{x}_\mathrm{act},\bm{v}_r,\bm{\omega}_r,\bm{u})
```
The physical actuator output (the output of the actuator dynamical system) is
```math
	\bm{\delta} = \bm{h}_\mathrm{act}(\bm{x}_\mathrm{act},\bm{v}_r,\bm{\omega}_r,\bm{u})
```


### Common Actuator Models
- No actuator dynamics: $\bm{\delta} = \bm{u}$
- First-order actuator model:
```math
	\begin{aligned}
		\dot{\bm{x}}_\mathrm{act} &= -\frac{1}{\tau} \bm{x}_\mathrm{act} + \frac{1}{\tau} \bm{u} \\
		\bm{\delta} &= \bm{x}_\mathrm{act}
	\end{aligned}
```
