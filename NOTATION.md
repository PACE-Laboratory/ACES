# [Notation and Variable Names](@id notation)

## Usage Rules
- This file is the canonical source for mathematical symbols and Julia identifiers.
- Source code should use the listed Julia identifier.
- ASCII fallbacks should be used only where Unicode identifiers are impractical.
- New public variables and mathematical operators must be added to this file.
- Definitions must state the coordinate frame whenever it is not unambiguous.
- Local module documentation should link to this file rather than duplicate entries.

## Notation and Variable Names

### Mathematical Symbols and Operators
| Mathematical notation | Julia identifier | ASCII fallback | Size | Description |
| --------------------- | ---------------- | -------------- | ---- | ----------- |
| $\mathbb{I}$ | `I` | `I` | Implied | Identity matrix |
| $\bm{S}(\cdot)$ | `cpem()` | `cpem()` | $\mathbb{R}^3 \to \mathfrak{so}(3)$ | Cross product equivalent matrix (hat map) |
| $\bm{S}^{-1}(\cdot)$ | `cpeminv()` | `cpeminv()` | $\mathfrak{so}(3) \to \mathbb{R}^3$ | Inverse map of the cross product equivalent matrix (vee map) |

### General Simulation Variables
| Mathematical notation | Julia identifier | ASCII fallback | Size | Description |
| --------------------- | ---------------- | -------------- | ---- | ----------- |
| $t$ | `t` | `t` | 1 | Time |
| $\bm{x}$ | `x` | `x` | $n_x$ (`n_x`) | Simulation (extended) state vector |

### Rigid-Body Dynamics
| Mathematical notation | Julia identifier | ASCII fallback | Size | Description |
| --------------------- | ---------------- | -------------- | ---- | ----------- |
| $m$ | `m` | `m` | 1 | Aircraft mass |
| $\bm{I}$ | `𝐈` | `MoI` | 3 × 3 | Aircraft moment of inertia matrix |
| $\bm{s}$ | `s` | `s` | 3 | Position of the body frame relative to the inertial frame, expressed in the inertial frame |
| $\bm{R}_\mathrm{IB}$ | `R_IB` | `R_IB` | 3 × 3 | Rotation matrix mapping vector components from the body frame to the inertial frame |
| $\bm{\Theta}$ | `Θ` | `Theta` | 3 | Vector of Euler angles |
| $\bm{q}_\mathrm{IB}$ | `q_IB` | `q_IB` | 4 | Unit quaternion corresponding to the rotation matrix $\bm{R}_\mathrm{IB}$ |
| $\bm{\eta}$ | `η` | `eta` | $n_\mathrm{pose}$ (`n_pose`) | Rigid body configuration vector |
| $\bm{v}$ | `v` | `v` | 3 | Inertial velocity expressed in the body frame |
| $\bm{\omega}$ | `ω` | `omega` | 3 | Angular velocity of the body frame with respect to the inertial frame, expressed in the body frame |
| $\bm{\nu}$ | `ν` | `nu` | 6 | Generalized velocity |
| $\bm{g}$ | `g` | `g` | 3 | Gravitational acceleration expressed in the inertial frame |
| $\bm{L}_\mathrm{IB}(\bm{\Theta})$ | `L_IB` | `L_IB` | 3 × 3 | Map from body angular velocity to Euler angle rates |
| $\bm{\Xi}(\bm{q}_\mathrm{IB})$ | `Ξ` | `Xi` | 4 × 3 | Map from body angular velocity to twice the unit quaternion rate |
| $\bm{\Omega}(\bm{\omega})$ | `Ω` | `Omega` | 4 × 4 | Quaternion rate matrix formed from body angular velocity |
| $\bm{f}_\eta$ | `f_η` | `f_eta` | $(\bm{\eta},\bm{\nu}) \to \mathbb{R}^{n_\mathrm{pose}}$ | Rigid body kinematics vector field |
| $\bm{J}_\eta(\bm{\eta})$ | `J_η` | `J_eta` | $n_\mathrm{pose}$ × 6 | Rigid body kinematics matrix |
| $\mathscr{M}$ | `ℳ` | `mass_matrix` | 6 × 6 | Generalized mass matrix |
| $\mathscr{F}$ | `ℱ` | `generalized_force` | 6 | Generalized force and moment vector |
| $\bm{f}_\nu$ | `f_ν` | `f_nu` | $(\bm{\eta},\bm{\nu}) \to \mathbb{R}^6$ | Unforced rigid body dynamic vector field |
| $\bm{x}_\mathrm{rb}$ | `x_rb` | `x_rb` | $n_\mathrm{rb}$ (`n_rb`) | Rigid body state vector |

### Wind Model
| Mathematical notation | Julia identifier | ASCII fallback | Size | Description |
| --------------------- | ---------------- | -------------- | ---- | ----------- |
| $\bm{W}$ | `W` | `W` | $\mathbb{R}^3 \times \mathbb{R} \to \mathbb{R}^3$ | Wind vector field |
| $\bm{w}$ | `w` | `w` | 3 | Apparent wind velocity |
| $\nabla\bm{W}$ | `∇W` | `grad_W` | $\mathbb{R}^3 \times \mathbb{R} \to \mathbb{R}^{3 \times 3}$ | Wind field gradient |
| $\bm{\Phi}_W$ | `Φ_W` | `Phi_W` | 3 × 3 | Full wind-field gradient transformed into the body frame |
| $\bm{\omega}_W$ | `ω_W` | `omega_W` | 3 | Angular velocity given by the skew-symmetric part of $\bm{\Phi}_W$ |
| $\bm{\omega}_w$ | `ω_w` | `omega_w` | 3 | Apparent angular velocity of the wind |
| $\bm{\Phi}_w$ | `Φ_w` | `Phi_w` | 3 × 3 | Apparent body-frame wind gradient |
| $\bm{v}_r$ | `v_r` | `v_r` | 3 | Air-relative velocity expressed in the body frame |
| $\bm{\omega}_r$ | `ω_r` | `omega_r` | 3 | Apparent air-relative angular velocity expressed in the body frame |
| $\bm{\varpi}$ | `ϖ` | `w_gen` | 6 | Generalized wind velocity |
| $\bm{\nu}_r$ | `ν_r` | `nu_r` | 6 | Generalized air-relative velocity |
| $\bm{x}_w$ | `x_w` | `x_w` | $n_w$ (`n_w`) | Wind model state |
| $\bm{W}_w$ | `W_w` | `W_w` | $m_w$ (`m_w`) | Wiener process driving the wind model |
| $\bm{f}_w$ | `f_w` | `f_w` | $(\cdot) \to \mathbb{R}^{n_w}$ | Wind-model drift vector field |
| $\bm{\sigma}_w$ | `σ_w` | `sigma_w` | $(\cdot) \to \mathbb{R}^{n_w \times m_w}$ | Wind-model diffusion matrix field |
| $\bm{h}_w$ | `h_w` | `h_w` | $(\cdot) \to \mathbb{R}^6$ | Wind-model output map |
| $\delta\bm{w}_b$ | `δw_b` | `delta_w_b` | 3 | Turbulent wind fluctuation expressed in the body frame |
| $\bar{\bm{w}}$ | `w_bar` | `w_bar` | 3 | Deterministic bulk wind expressed in the inertial frame |
| $\bm{\xi}_w$ | `ξ_w` | `xi_w` | 6 | Unit-variance continuous-time white-noise input to a wind shaping filter |

### Aerodynamic Model
| Mathematical notation | Julia identifier | ASCII fallback | Size | Description |
| --------------------- | ---------------- | -------------- | ---- | ----------- |
| $\bm{F}$ | `F` | `F` | 3 | Aerodynamic force |
| $\bm{M}$ | `M` | `M` | 3 | Aerodynamic moment |
| $\bm{F}_\mathrm{model}$ | `F_model` | `F_model` | $(\cdot) \to \mathbb{R}^3$ | Aerodynamic force model |
| $\bm{M}_\mathrm{model}$ | `M_model` | `M_model` | $(\cdot) \to \mathbb{R}^3$ | Aerodynamic moment model |
| $\bm{x}_\mathrm{aero}$ | `x_aero` | `x_aero` | $n_\mathrm{aero}$ (`n_aero`) | Unsteady aerodynamic model state |
| $\bm{f}_\mathrm{aero}$ | `f_aero` | `f_aero` | $(\cdot) \to \mathbb{R}^{n_\mathrm{aero}}$ | Unsteady aerodynamic model vector field |
| $\bm{x}_\mathrm{res}$ | `x_res` | `x_res` | $n_\mathrm{res}$ (`n_res`) | Aerodynamic model residual state |
| $\bm{W}_\mathrm{res}$ | `W_res` | `W_res` | $m_\mathrm{res}$ (`m_res`) | Wiener process driving the aerodynamic residual model |
| $\delta \bm{F}$ | `δF` | `delta_F` | 3 | Force model residual |
| $\delta \bm{M}$ | `δM` | `delta_M` | 3 | Moment model residual |
| $\bm{f}_\mathrm{res}$ | `f_res` | `f_res` | $(\cdot) \to \mathbb{R}^{n_\mathrm{res}}$ | Aerodynamic model residual drift vector field |
| $\bm{\sigma}_\mathrm{res}$ | `σ_res` | `sigma_res` | $(\cdot) \to \mathbb{R}^{n_\mathrm{res} \times m_\mathrm{res}}$ | Aerodynamic model residual diffusion matrix field |
| $\bm{g}_\mathrm{res}$ | `g_res` | `g_res` | $(\cdot) \to \mathbb{R}^6$ | Output function of the aerodynamic residual model |
| $V$ | `V` | `V` | 1 | Airspeed |
| $\alpha$ | `α` | `alpha` | 1 | Angle of attack |
| $\beta$ | `β` | `beta` | 1 | Sideslip angle |

### Actuator Model
| Mathematical notation | Julia identifier | ASCII fallback | Size | Description |
| --------------------- | ---------------- | -------------- | ---- | ----------- |
| $\bm{f}_{act}$ | `f_act` | `f_act` | $(\cdot) \to \mathbb{R}^{n_\mathrm{act}}$ | Actuator dynamics vector field |
| $\bm{x}_\mathrm{act}$ | `x_act` | `x_act` | $n_\mathrm{act}$ (`n_act`) | Actuator dynamical system state |
| $\bm{u}$ | `u` | `u` | $p_\mathrm{act}$ (`p_act`) | Actuator input |
| $\bm{\delta}$ | `δ` | `delta` | $p_\mathrm{act}$ (`p_act`) | Physical actuator output or configuration |
| $\bm{h}_\mathrm{act}$ | `h_act` | `h_act` | $(\cdot) \to \mathbb{R}^{p_\mathrm{act}}$ | Actuator output map |

### Control Model
| Mathematical notation | Julia identifier | ASCII fallback | Size | Description |
| --------------------- | ---------------- | -------------- | ---- | ----------- |
| $\bm{x}_\mathrm{ctrl}$ | `x_ctrl` | `x_ctrl` | $n_\mathrm{ctrl}$ (`n_ctrl`) | Controller state |
| $\bm{f}_\mathrm{ctrl}$ | `f_ctrl` | `f_ctrl` | $(\cdot) \to \mathbb{R}^{n_\mathrm{ctrl}}$ | Controller dynamics vector field |
| $\bm{h}_\mathrm{ctrl}$ | `h_ctrl` | `h_ctrl` | $(\cdot) \to \mathbb{R}^{p_\mathrm{ctrl}}$ | Controller output map |
| $\delta\bm{u}$ | `δu` | `delta_u` | $p_\mathrm{ctrl}$ (`p_ctrl`) | Control-input perturbation from a reference value |
| $\delta\bm{x}$ | `δx` | `delta_x` | $n_x$ (`n_x`) | State perturbation from a reference value |
| $\delta\bm{y}$ | `δy` | `delta_y` | $n_y$ (`n_y`) | Output perturbation from a reference value |
| $\bm{u}_\mathrm{ref}$ | `u_ref` | `u_ref` | $p_\mathrm{ctrl}$ (`p_ctrl`) | Reference control input |
| $\bm{x}_\mathrm{ref}$ | `x_ref` | `x_ref` | $n_x$ (`n_x`) | Reference state |
| $\bm{y}$ | `y` | `y` | $n_y$ (`n_y`) | Measured output for feedback control |
| $\bm{y}_\mathrm{ref}$ | `y_ref` | `y_ref` | $n_y$ (`n_y`) | Reference output |
| $\bm{K}$ | `K` | `K` | $p_\mathrm{ctrl} \times n_x$ | Linear static state feedback gain matrix |
| $\bm{A}_\mathrm{ctrl},\bm{B}_\mathrm{ctrl},\bm{C}_\mathrm{ctrl},\bm{D}_\mathrm{ctrl}$ | `A_ctrl`, `B_ctrl`, `C_ctrl`, `D_ctrl` | Same as Julia | Compatible | State space matrices of a linear dynamic output feedback controller |

### Estimation Model
| Mathematical notation | Julia identifier | ASCII fallback | Size | Description |
| --------------------- | ---------------- | -------------- | ---- | ----------- |
| $\bm{x}_\mathrm{meas}$ | `x_meas` | `x_meas` | $n_\mathrm{meas}$ (`n_meas`) | Measurement model state |
| $\bm{y}_\mathrm{meas}$ | `y_meas` | `y_meas` | $p_\mathrm{meas}$ (`p_meas`) | Measurement model output |
| $\bm{W}_\mathrm{meas}$ | `W_meas` | `W_meas` | $m_\mathrm{meas}$ (`m_meas`) | Wiener process driving the measurement model |
| $\bm{f}_\mathrm{meas}$ | `f_meas` | `f_meas` | $(\cdot) \to \mathbb{R}^{n_\mathrm{meas}}$ | Measurement model drift vector field |
| $\bm{\sigma}_\mathrm{meas}$ | `σ_meas` | `sigma_meas` | $(\cdot) \to \mathbb{R}^{n_\mathrm{meas} \times m_\mathrm{meas}}$ | Measurement model diffusion matrix field |
| $\bm{h}_\mathrm{meas}$ | `h_meas` | `h_meas` | $(\cdot) \to \mathbb{R}^{p_\mathrm{meas}}$ | Measurement model output map |
| $\bm{x}_\mathrm{est}$ | `x_est` | `x_est` | $n_\mathrm{est}$ (`n_est`) | Estimator state |
| $\hat{\bm{x}}$ | `x_hat` | `x_hat` | $p_\mathrm{est}$ (`p_est`) | Extended state estimate |
| $\bm{f}_\mathrm{est}$ | `f_est` | `f_est` | $(\cdot) \to \mathbb{R}^{n_\mathrm{est}}$ | Estimator dynamics vector field used by the simulation |
| $\bm{h}_\mathrm{est}$ | `h_est` | `h_est` | $(\cdot) \to \mathbb{R}^{p_\mathrm{est}}$ | Estimator output map used by the simulation |

### Environment Model
| Mathematical notation | Julia identifier | ASCII fallback | Size | Description |
| --------------------- | ---------------- | -------------- | ---- | ----------- |
| $h$ | `h` | `h` | 1 | Altitude above mean sea level |
| $h_0$ | `h0` | `h0` | 1 | Altitude above mean sea level of the NED-frame origin |
| $g$ | `g` | `g` | 1 | Gravitational acceleration magnitude |
| $T$ | `T` | `T` | 1 | Atmospheric temperature |
| $p$ | `p` | `p` | 1 | Atmospheric pressure |
| $\rho$ | `ρ` | `rho` | 1 | Atmospheric density |
| $\rho_0$ | `ρ0` | `rho0` | 1 | Reference atmospheric density |
| $h_\mathrm{scale}$ | `h_scale` | `h_scale` | 1 | Atmospheric density scale height |
