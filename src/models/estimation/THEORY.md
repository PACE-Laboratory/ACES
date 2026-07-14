# Theory: Estimation

## Assumptions
- The measured output is almost surely continuous and does not admit perturbation by addititve white noise
- The estimator dynamics are continuous-time with no delay
- The estimator output has no delay and is causal

## Inputs and Outputs

**Inputs:**
- Measurement model state, $\bm{x}_\mathrm{meas} \in \mathbb{R}^{n_\mathrm{meas}}$
- Estimator state, $\bm{x}_\mathrm{est} \in \mathbb{R}^{n_\mathrm{est}}$
- Simulation state, $\bm{x} \in \mathbb{R}^{n_x}$
- Control input, $\bm{u} \in \mathbb{R}^{p_\mathrm{ctrl}}$
- Time, $t \in \mathbb{R}_+$

**Outputs:**
- Extended state estimate $\hat{\bm{x}} \in \mathbb{R}^{p_\mathrm{est}}$
- Measurement model state time derivative, $\dot{\bm{x}}_\mathrm{meas} \in \mathbb{R}^{n_\mathrm{meas}}$
- Estimator state time derivative, $\dot{\bm{x}}_\mathrm{est} \in \mathbb{R}^{n_\mathrm{est}}$

## Measurement Model

The measured output $\bm{y}_\mathrm{meas} \in \mathbb{R}^{p_\mathrm{meas}}$ of a system may be assumed to satisfy either
$$
	\bm{y}_\mathrm{meas} = \bm{h}_\mathrm{meas}(\bm{x},\bm{u},t) + \tilde{\bm{v}}_\mathrm{meas}
$$
where $\tilde{\bm{v}}_\mathrm{meas} \in \mathbb{R}^{m_\mathrm{est}}$ is the measurment disturbance (typically assumed to be zero-mean Gaussian white noise), or more formally, the stochastic input-output system
$$
	\begin{aligned}
		\mathrm{d}\bm{x}_\mathrm{meas} &= \bm{f}_\mathrm{meas}(\bm{x}_\mathrm{meas},\bm{x},\bm{u},t) \mathrm{d}t + \bm{\sigma}_\mathrm{meas} (\bm{x}_\mathrm{meas},\bm{x},\bm{u},t) \mathrm{d}\bm{W}_\mathrm{meas} \\
		\bm{y}_\mathrm{meas} &= \bm{h}_\mathrm{meas}(\bm{x}_\mathrm{meas},\bm{x},\bm{u},t)
	\end{aligned}
$$
where $\bm{x}_\mathrm{meas} \in \mathbb{R}^{n_\mathrm{meas}}$ is the measurement-model state and $\bm{W}_\mathrm{meas}$ is a Wiener process defined on $\mathbb{R}^{m_\mathrm{meas}}$. Although this more formal formulation does not permit additive white noise to enter $\bm{y}_\mathrm{meas}$ directly, it can pose difficulties for numerical integration schemes and is therefore avoided. Moreover, real sensors and low-level navigation solutions are not directly perturbed by white noise.

## State Estimator System
In general, consider the state and/or disturbance estimation dynamical system
$$
	\dot{\bm{x}}_\mathrm{est} = \bm{f}'_\mathrm{est}(\bm{x}_\mathrm{est},\bm{y}_\mathrm{meas},\bm{u}_\mathrm{est},t)
$$
where $\bm{x}_\mathrm{est} \in \mathbb{R}^{n_est}$ is the state of the estimator and $\bm{u}_\mathrm{est} \in \mathbb{R}^{m_est}$ is the input (known signal) for the purpose of designing and implementing the state estimation scheme. For simulation, we assume the mapping $(\bm{x},\bm{u})\mapsto \bm{u}_\mathrm{est}$ is implmented in the estimator dynamics model such that 
$$
	\dot{\bm{x}}_\mathrm{est} = \bm{f}_\mathrm{est}(\bm{x}_\mathrm{est},\bm{y}_\mathrm{meas},\bm{x},\bm{u},t)
$$
The extended state estimate $\hat{\bm{x}} \in \mathbb{R}^{p_\mathrm{est}}$ (the output of the state estimation system) satisfies
$$
	\hat{\bm{x}} = \bm{h}_\mathrm{est}'(\bm{x}_\mathrm{est},\bm{y}_\mathrm{meas},\bm{u}_\mathrm{est},t)
$$
which need not be an estimate of the entire state vector $\bm{x}$. For simulation, we assume 
$$
	\hat{\bm{x}} = \bm{h}_\mathrm{est}(\bm{x}_\mathrm{est},\bm{y}_\mathrm{meas},\bm{x},\bm{u},t)
$$

## Relationship to the Extended Kalman Filter

Since our formulation does not admit true white measurement noise, we instead consider the measurement model
$$
	\begin{aligned}
		\mathrm{d}\bm{x}_\mathrm{meas} &= -\frac{1}{\tau} \bm{x}_\mathrm{meas} \mathrm{d}t + \frac{1}{\tau}\sqrt{\bm{R}} \, \mathrm{d}\bm{W}_\mathrm{meas} \\
		\bm{y}_\mathrm{meas} &= \bm{h}(\bm{x}) + \bm{x}_\mathrm{meas}
	\end{aligned}
$$
As $\tau \to 0$, $\bm{x}_\mathrm{meas}$ approaches white noise with power spectral density $\bm{R}$ in the distributional sense. 

Let the estimator state be
$$
	\bm{x}_\mathrm{est} = \begin{bmatrix} 
		\hat{\bm{x}} \\
		\mathrm{vex}(\bm{P})
	\end{bmatrix}
$$
where $\mathrm{vex}(\bm{P})$ is the vectorization of state matrix $\bm{P}$ of the differential Riccati equation.