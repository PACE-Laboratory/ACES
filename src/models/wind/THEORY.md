# Wind Model Theory

Symbols and Julia identifiers follow the [canonical notation](@ref notation).

## Inputs and Outputs

**Inputs:**
- Wind model state, $\bm{x}_w \in \mathbb{R}^{n_w}$
- Rigid body state, $\bm{x}_\mathrm{rb} \in \mathbb{R}^{n_\mathrm{rb}}$ 
- Time, $t \in \mathbb{R}_+$

**Outputs:**
- Generalized wind velocity, $\bm{\varpi} \in \mathbb{R}^6$
- Wind model state time derivative, $\dot{\bm{x}}_w \in \mathbb{R}^{n_w}$

## Wind Field

In general and independent of the aircraft's motion, wind is a time-varying vector field, $\bm{W}:\mathbb{R}^3\times\mathbb{R}\to\mathbb{R}^3$, defined in the inertial frame. Let the instantaneous wind vector as experienced by the aircraft be
```math
	\bm{w}(t) = \bm{W}(\bm{s}(t),t)
```
The apparent wind $\bm{w}$ is defined by evaluating the wind field $\bm{W}$ at the aircraft's position $\bm{s}$ at time $t$ as if the aircraft were absent. Using the chain rule, the time derivative of $\bm{w}$ is
```math
	\frac{\mathrm{d}\bm{w}}{\mathrm{d}t} = \frac{\partial\bm{W}}{\partial t}(\bm{s},t) + \nabla\bm{W}(\bm{s},t) \frac{\mathrm{d}\bm{s}}{\mathrm{d}t}
```
Note that we have arrived at this expression under the implicit assumption that the vehicle does not affect the flow field in which it is immersed. When the aircraft's velocity through a wind field is significantly faster than the time rate of change of the eddies (such as for fixed-wing aircraft), one can make a frozen turbulence assumption, meaning $\frac{\partial\bm{W}}{\partial t}(\bm{s},t) = \bm{0}$ and the aparent wind velocity time derivative becomes
```math
	\dot{\bm{w}} = \nabla\bm{W}(\bm{s}) \dot{\bm{s}}
```
Conversely, if the aircraft is not moving, the eddies are being convected over the aircraft by the bulk flow. Therefore, $\dot{\bm{w}}$ would instead be
```math
	\dot{\bm{w}} = \frac{\partial\bm{W}(t)}{\partial t}
```
where $\frac{\partial\bm{W}}{\partial t}$ is the time rate of change of wind velocity at the aircraft's position due to the convected eddies.

## Wind Field Experienced by the Aircraft

The above models for the apparent wind treat the aircraft as a single point in the wind field. However, the gradient of the wind field on the scale of the aircraft also influences its motion. Independent of the vehicle's geometry, the body-frame wind field gradient for the aircraft's current pose ($\bm{s}$ and $\bm{R}_\mathrm{IB}$ at time $t$) is
```math
	\bm{\Phi}_W(\bm{s},\bm{R}_\mathrm{IB},t) = \bm{R}_\mathrm{IB}^\mathsf{T} \nabla \bm{W}(\bm{s},t) \bm{R}_\mathrm{IB}
```
The body-frame gradient of the wind field can be decomposed into its symmetric and skew-symmetric parts,
```math
	\bm{\Phi}_W = \frac{1}{2}(\bm{\Phi}_W + \bm{\Phi}_W^\mathsf{T}) + \frac{1}{2}(\bm{\Phi}_W - \bm{\Phi}_W^\mathsf{T})
```
The skew-symmetric part $\frac{1}{2}(\bm{\Phi}_W - \bm{\Phi}_W^\mathsf{T})$ defines the angular velocity of the wind in the body frame. As a vector-valued map,
```math
	\bm{\omega}_W = \frac{1}{2}\bm{S}^{-1}(\bm{\Phi}_W - \bm{\Phi}_W^\mathsf{T})
```
where $\bm{S}^{-1}(\cdot)$ denotes the inverse of the mapping $\bm{S}(\cdot)$; that is, $\bm{S}^{-1}(\bm{S}(\bm{a})) = \bm{a} \in \mathbb{R}^3$. While $\bm{\omega}_W$ is in fact a vector field assigning a body-frame wind angular velocity to each time $t$ and aircraft configuration $(\bm{s},\bm{R}_\mathrm{IB})$, Etkin explains that each aircraft will experience a different gradient based on its geometry. To illustrate this fact, consider an arbitrary point $\bm{r} = [x_b ~ y_b ~ z_b]^\mathsf{T}$ defined relative to the CG in the body frame. At an instant in time,
```math
	\bm{W}_b(\bm{r},t) = \begin{bmatrix}
		u_W(x_b,y_b,z_b,t) \\ v_W(x_b,y_b,z_b,t) \\ w_W(x_b,y_b,z_b,t)
	\end{bmatrix} = \bm{R}_\mathrm{IB}^\mathsf{T}(t) \bm{W}(\bm{s}(t) + \bm{R}_\mathrm{IB}(t)\bm{r},t)
```
is the body-frame wind field. In the case of traditional fixed-wing aircraft, for example, Etkin argues the only non-negligible gradients of the body-frame wind field $\bm{W}_b$ are
```math
	p_w = \frac{\partial w_W}{\partial y_b} ,~ q_w = -\frac{\partial w_W}{\partial x_b} ,~\text{and}~ r_w = \frac{\partial v_W}{\partial x_b}
```
reflecting an assumption that
- the body-longitudinal component of wind, $u_W$, is constant/uniform over the entire aircraft,
- the body-lateral component of wind, $v_W$, varies only along the aircraft's length, and
- the body-vertical component of wind, $w_W$, varies only along the aircraft's length and span.

The vector $\bm{\omega}_w = [p_w ~ q_w ~ r_w]^\mathsf{T}$ as defined above is called the apparent angular velocity of the wind. Therefore, a fixed-wing aircraft experiences the apparent body-frame wind gradient
```math
	\bm{\Phi}_w = \begin{bmatrix}
		0 & 0 & 0 \\ 
		r_w & 0 & 0 \\ 
		-q_w & p_w & 0
	\end{bmatrix}
```
Note the difference in subscript as compared to the skew-symmetric part of $\bm{\Phi}_W$. For other types of aircraft, $\bm{\Phi}_w$ may take on a different structure, and $\bm{\omega}_w$ will be defined accordingly. In general, the relationship between the body-frame wind field gradient $\bm{\Phi}_W$ and the apparent body frame wind gradient $\bm{\Phi}_w$ depends on the aircraft's geometry and aerodynamics.

For conveinence, we define the generalzied wind velocity to be
```math
	\bm{\varpi} = \begin{bmatrix}
		\bm{w} \\ 
		\bm{\omega}_w
	\end{bmatrix}
```

## Wind Triangle
Let 
```math
	\bm{v}_r = \bm{v} - \bm{R}_\mathrm{IB}^\mathsf{T} \bm{w}
```
be the air-relative translational velocity, and let
```math
	\bm{\omega}_r = \bm{\omega} - \bm{\omega}_w
```
be the apparent air-relative angular velocity.

## Wind Model

In general, we consider the wind model dynamics
```math
	\mathrm{d}\bm{x}_w = \bm{f}_w(\bm{x}_w,\bm{x}_\mathrm{rb},t) \mathrm{d}t + \bm{\sigma}_w(\bm{x}_w,\bm{x}_\mathrm{rb},t) \mathrm{d}\bm{W}_w
```
where $\bm{x}_w \in \mathbb{R}^{n_w}$ is the state of the wind model and $\bm{W}_w$ is a Wiener process on $\mathbb{R}^{m_w}$. Then output of the wind model, $\bm{\varpi}$, satisfies
```math
	\bm{\varpi} = \bm{h}_w(\bm{x}_w,\bm{x}_\mathrm{rb},t)
```

### Shaping Filter Models

Dryden and von Kármán turbulence models aim to characterize the frequency content of the body-frame wind fluctuations $\delta\bm{w}_b$ and the body-frame apparent angular velocity $\bm{\omega}_w$. These fluctuations satisfy a Reynolds decomposition,
```math
	\bm{w} = \bar{\bm{w}} + \bm{R}_\mathrm{IB} \delta\bm{w}_b
```
where $\bar{\bm{w}}$ is the deterministic bulk flow. With a state space realization, Dryden and approximations of von Kármán turbulence models act as noise shaping filters and take the general form
```math
	\begin{aligned}
		\dot{\bm{x}}_w &= \bm{A}_w \bm{x}_w + \bm{B}_w \bm{\xi}_w \\
		\delta\bm{w}_b &= \bm{C}_{\delta w_b} \bm{x}_w \\
		\bm{\omega}_w  &= \bm{C}_{\omega_w} \bm{x}_w
	\end{aligned}
```
where $\bm{\xi}_w$ is 6-dimensional continuous-time white noise with unit variance.


### Frozen Wind Field Model

If the wind field $\bm{W}$ is known analytically or numerically, then there are no wind model dynamics and thus $\bm{w}$ and $\bm{\omega}_w$ can be directly computed.
