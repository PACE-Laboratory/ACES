# Theory: Rigid Body Dynamics

## Reference Frames

Let the orthonormal vectors $\{\bm{i}_1, \bm{i}_2, \bm{i}_3\}$ define an earth-fixed North-East-Down (NED) reference frame, $\mathcal{F}_\mathrm{I}$, which we take to be inertial over the time and space scales of motion considered. Let the orthonormal vectors $\{\bm{b}_1, \bm{b}_2, \bm{b}_3\}$ define the body-fixed frame, $\mathcal{F}_\mathrm{B}$, centered at the aircraft's center of gravity (CG), with $\bm{b}_1$ pointing out the front of the aircraft, $\bm{b}_2$ pointing out the right-hand side, and $\bm{b}_3$ pointing downward to complete a right-handed frame. 

## Rigid Body Configuration (Pose)
The position of the body frame with respect to the inertial frame is given by the vector $\bm{s} = [s_N~s_E~s_D]^\mathsf{T}$. The attitude of the aircraft is described by the rotation matrix $\bm{R}_\mathrm{IB}$ that maps vector components from $\mathcal{F}_\mathrm{B}$ to $\mathcal{F}_\mathrm{I}$. The matrix $\bm{R}_\mathrm{IB}$ is an element of the special orthogonal group,
$$
	\mathsf{SO}(3) = \{ \bm{R} \in \mathbb{R}^{3 \times 3} \mid \bm{R}^{-1} = \bm{R}^\mathsf{T} ,~ \det\bm{R} = 1 \}
$$
with the Lie group action being matrix multiplication. The aircraft's configuration is described by points $(\bm{s},\bm{R}_\mathrm{IB})$ in the special Euclidean group, 
$$
	\mathsf{SE}(3) = \mathbb{R}^3 \rtimes \mathsf{SO}(3)
$$
where $\rtimes$ is the semi-direct product, which expresses how two elements of the group compose a new element. 

### Euler Angles

To fascilliate local analysis in Euclidean space, we will consider local parameterizations of $\mathsf{SO}(3)$. Most common are the roll-pitch-yaw Euler angles, which parameterize $\bm{R}_\mathrm{IB}$ as three successive rotations about basis vectors. Let $\bm{S}(\cdot)$ be the skew-symmetric cross product equivalent matrix operator satisfying $\bm{S}(\bm{a})\bm{b} = \bm{a} \times \bm{b}$ for 3-vectors $\bm{a}$ and $\bm{b}$. Written out, for $\bm{a}=[a_1~a_2~a_3]^\mathsf{T}\in\mathbb{R}^3$, the cross-product equivalent matrix is
$$
   \bm{S}(\bm{a}) = \begin{bmatrix}
      0 & -a_3 & a_2\\
      a_3 & 0 & -a_1\\
      -a_2 & a_1 & 0
   \end{bmatrix}
$$
The fundamental active rotations are defined as
$$
	\begin{gathered}
		\bm{R}_1(\alpha) = e^{\bm{S}(\bm{e}_1)\alpha} = \begin{bmatrix}
			1 & 0 & 0 \\
			0 & \cos\alpha & -\sin\alpha \\
			0 & \sin\alpha & \cos\alpha
		\end{bmatrix} \\
		\bm{R}_2(\alpha) = e^{\bm{S}(\bm{e}_2)\alpha} = \begin{bmatrix}
			\cos\alpha & 0 & \sin\alpha \\
			0 & 1 & 0 \\
			-\sin\alpha & 0 & \cos\alpha
		\end{bmatrix} \\
		\bm{R}_3(\alpha) = e^{\bm{S}(\bm{e}_3)\alpha} = \begin{bmatrix}
			\cos\alpha & -\sin\alpha & 0 \\
			\sin\alpha & \cos\alpha & 0 \\
			0 & 0 & 1
		\end{bmatrix}
	\end{gathered}
$$
where $\bm{e}_1 = [1~0~0]^\mathsf{T}$, $\bm{e}_2 = [0~1~0]^\mathsf{T}$, and $\bm{e}_3 = [0~0~1]^\mathsf{T}$ form the standard Euclidean basis. We will parameterize $\bm{R}_\mathrm{IB}$ using the 1-2-3 Euler angle sequence
$$
	\bm{R}_\mathrm{IB}(\bm{\Theta}) = \bm{R}_3(\psi) \bm{R}_2(\theta) \bm{R}_1(\phi)
$$
where $\phi$, $\theta$, and $\psi$ are the roll, pitch, and yaw angles of the aircraft, respectively, which are colleted in the column vector $\bm{\Theta} = [\phi~\theta~\psi]^\mathsf{T}$. Written out,
$$
	\bm{R}_\mathrm{IB}(\bm{\Theta}) = \begin{bmatrix}
		\cos\psi\cos\theta & \cos\psi\sin\phi\sin\theta - \cos\phi\sin\psi & \sin\phi\sin\psi + \cos\phi\cos\psi\sin\theta \\
		\cos\theta\sin\psi & \cos\phi\cos\psi + \sin\phi\sin\psi\sin\theta & \cos\phi\sin\psi\sin\theta - \cos\psi\sin\phi \\
		-\sin\theta & \cos\theta\sin\phi & \cos\phi\cos\theta
	\end{bmatrix}
$$

### Unit Quaternions

Consider two quaternions $q = q_0 + q_1 \hat{i} + q_2 \hat{j} + q_3 \hat{k}$ and $p = p_0 + p_1 \hat{i} + p_2 \hat{j} + p_3 \hat{k}$, represented as the column vectors
$$
	\bm{q} = \begin{bmatrix}
		q_0 \\ q_1 \\ q_2 \\ q_3
	\end{bmatrix} = \begin{bmatrix}
		q_0 \\ \vec{\bm{q}}
	\end{bmatrix}
	\quad\text{and}\quad
	\bm{p} = \begin{bmatrix}
		p_0 \\ p_1 \\ p_2 \\ p_3
	\end{bmatrix} = \begin{bmatrix}
		p_0 \\ \vec{\bm{p}}
	\end{bmatrix}
$$
The magnitude of $q \cong \bm{q}$ is 
$$
   |q| = \|\bm{q}\| = \sqrt{q_0^2 + q_1^2 + q_2^2 + q_3^2}
$$
Quaternion addition is simply $ q + p = \bm{q}+\bm{p}$. Quaternion multiplication satisfies
$$
	\bm{q} \ast \bm{p} = \begin{bmatrix}
		q_0 p_0 - \vec{\bm{q}}\cdot\vec{\bm{p}} \\ q_0 \vec{\bm{p}} + p_0 \vec{\bm{q}} + \vec{\bm{q}}\times\vec{\bm{p}}
	\end{bmatrix} = \begin{bmatrix}
		q_0 p_0 - q_1 p_1 - q_2 p_2 - q_3 p_3 \\
		q_0 p_1 + q_1 p_0 + q_2 p_3 - q_3 p_2 \\
		q_0 p_2 - q_1 p_3 + q_2 p_0 + q_3 p_1 \\ 
		q_0 p_3 + q_1 p_2 - q_2 p_1 + q_3 p_0
	\end{bmatrix}
$$
and quaternion conjugation is denoted
$$
	\begin{gathered}
		q^\ast = q_0 - q_1 \hat{i} - q_2 \hat{j} - q_3 \hat{k} \\
		\bm{q}^\ast = \begin{bmatrix} q_0 \\ -\vec{\bm{q}} \end{bmatrix}
	\end{gathered}
$$

Let $\bm{q}_\mathrm{IB} = \bm{q}$ as defined above. Then, the rotation matrix $\bm{R}_\mathrm{IB}$ is parameterized as
$$
	\bm{R}_\mathrm{IB}(\bm{q}_\mathrm{IB}) = (q_0^2 - \vec{\bm{q}}^\mathsf{T}\vec{\bm{q}})\mathbb{I} + 2\vec{\bm{q}}\vec{\bm{q}}^\mathsf{T} + 2 q_0 \bm{S}(\vec{\bm{q}}) 
$$
Written out,
$$
	\bm{R}_\mathrm{IB}(\bm{q}_\mathrm{IB}) = \begin{bmatrix}
		q_0^2 + q_1^2 - q_2^2 - q_3^2 &         2 q_1 q_2 - 2 q_0 q_3 &         2 q_0 q_2 + 2 q_1 q_3 \\
		2 q_0 q_3 + 2 q_1 q_2 & q_0^2 - q_1^2 + q_2^2 - q_3^2 &         2 q_2 q_3 - 2 q_0 q_1 \\
		2 q_1 q_3 - 2 q_0 q_2 &         2 q_0 q_1 + 2 q_2 q_3 & q_0^2 - q_1^2 - q_2^2 + q_3^2
	\end{bmatrix}
$$
The unit quaternions are a double covering of $\mathsf{SO}(3)$. For any rotation,
$$
	\bm{R}(\bm{q}) = \bm{R}(-\bm{q})
$$
The unit quaternion components that correspond to the rotation matrix $\bm{R}_\mathrm{IB} = [r_{ij}]$ are
$$
   \begin{aligned}
	   q_0 &= \pm \frac{1}{2}\sqrt{1 + \operatorname{Tr}(\bm{R}_\mathrm{IB})} \\
	   q_1 &= \frac{r_{32}-r_{23}}{4 q_0} \\
	   q_2 &= \frac{r_{13}-r_{31}}{4 q_0} \\
	   q_3 &= \frac{r_{21}-r_{12}}{4 q_0}
   \end{aligned}
$$
We will always choose $q_0$ to be positive to enfore a one-to-one correspodence between the attitude quaternion and the attitude rotation matrix. That is,
$$
	q_0 = + \frac{1}{2}\sqrt{1 + \operatorname{Tr}(\bm{R}_\mathrm{IB})}
$$

In terms of quaternion operations, the composition of two rotations is $\bm{q}_\mathrm{AC} = \bm{q}_\mathrm{AB} \ast \bm{q}_\mathrm{BC}$ and an invserse rotation is $\bm{q}_\mathrm{BI} = \bm{q}_\mathrm{IB}^\ast$.

### Abstract Configuration Representation

Let $\bm{\eta} \in \mathbb{R}^{n_\mathrm{pose}}$ denote the chosen rigid-body configuration representaion, which locally (away from singularities) means that
$$
	\begin{gathered}
		\bm{\eta} = \begin{bmatrix} \bm{s} \\ \bm{\Theta} \end{bmatrix} \leftrightarrow (\bm{s},\bm{R}_\mathrm{IB}) \\
		\bm{\eta} = \begin{bmatrix} \bm{s} \\ \bm{q}_\mathrm{IB} \end{bmatrix} \leftrightarrow (\bm{s},\bm{R}_\mathrm{IB})
	\end{gathered}
$$

## Rigid Body Velocity

Let $\bm{v} = [u~v~w]^\mathsf{T} \in \mathbb{R}^3$ be the inertial velocity of the rigid body, expressed in the body frame. Let $\bm{\omega} = [p~q~r]^\mathsf{T} \in \mathbb{R}^3$ be the angular velocity of the body frame with respect to the inertial frame, expressed in the body frame. Let $\bm{\nu} = (\bm{v},\bm{\omega}) \in \mathbb{R}^6$ denote the generalized velocity of the rigid body. 

The matrix $\bm{S}(\bm{\omega})$ is an element of the Lie algebra $\mathfrak{so}(3)$ of skew-symmetric $3 \times 3$ matrices. Through the cross product equivalent matrix, $\mathfrak{so}(3)$ is directly related to $\mathbb{R}^3$. That is,
$$
	\bm{S}: \mathbb{R}^3 \to \mathfrak{so}(3) \quad\text{and}\quad \bm{S}^{-1}: \mathfrak{so}(3) \to \mathbb{R}^3
$$

## Rigid Body Kinematics

The rigid body translational kinematics are
$$
	\dot{\bm{s}} = \bm{R}_\mathrm{IB} \bm{v}
$$

The rigid body rotational kinematics are fundamentally written as
$$
	\dot{\bm{R}}_\mathrm{IB} = \bm{R}_\mathrm{IB} \bm{S}(\bm{\omega})
$$
Using the 1-2-3 (roll-pitch-yaw) Euler angle parameterization, we have
$$
	\dot{\bm{\Theta}} = \bm{L}_\mathrm{IB}(\bm{\Theta}) \bm{\omega}
$$
where
$$
	\bm{L}_\mathrm{IB}(\bm{\Theta}) = \begin{bmatrix}
			1 & \sin\phi\tan\theta 	& \cos\phi\tan\theta \\
			0 & \cos\phi 			& -\sin\phi \\
			0 & \sin\phi\sec\theta 	& \cos\phi\sec\theta
	\end{bmatrix}
$$
which reveals a singularity when $\theta = \pm \pi/2$.

Using unit quaternions, we have
$$
	\dot{\bm{q}}_\mathrm{IB} = \frac{1}{2}\bm{\Xi}(\bm{q}_\mathrm{IB})\bm{\omega}
$$
where
$$
	\bm{\Xi}(\bm{q}_\mathrm{IB}) = \begin{bmatrix}
		-\vec{\bm{q}}^\mathsf{T} \\
		q_0 \mathbb{I} + \bm{S}(\vec{\bm{q}})
	\end{bmatrix} = \begin{bmatrix}
		-q_1 & -q_2 & -q_3 \\
		q_0 & -q_3 & q_2 \\
		q_3 & q_0 & -q_1 \\
		-q_2 & q_1 & q_0
	\end{bmatrix}
$$
or equivalently,
$$
	\dot{\bm{q}}_\mathrm{IB} = \frac{1}{2}\bm{\Omega}(\bm{\omega}) \bm{q}_\mathrm{IB}
$$
where
$$
	\bm{\Omega}(\bm{\omega}) = \begin{bmatrix}
		0 & -\bm{\omega}^\mathsf{T} \\
		\bm{\omega} & -\bm{S}(\bm{\omega})
	\end{bmatrix} = \begin{bmatrix}
		0 & -p & -q & -r \\
		p & 0 & r & -q \\
		q & -r & 0 & p \\
		r & q & -p & 0
	\end{bmatrix}
$$

Putting together the translational kinematics and rotational kinematics, the aircraft kinematics are written compactly as
$$
	\dot{\bm{\eta}} = \bm{f}_\eta(\bm{\eta},\bm{\nu})
$$
or equivalently,
$$
	\dot{\bm{\eta}} = \bm{J}_\eta(\bm{\eta})\bm{\nu}
$$
where $\bm{J}_\eta(\bm{\eta}) \in \mathbb{R}^{n_\mathrm{pose} \times 6}$.

## Rigid Body Dynamics

Let $\bm{F} \in \mathbb{R}^3$ and $\bm{M} \in \mathbb{R}^3$ respectively denote the aerodynamic force and moment acting at the aircraft CG, expressed in the body frame. Let $\bm{g}$ be the gravitational acceleration vector in the inertial frame. Suppose the aircraft's mass $m$ is constant, and let $\bm{I}$ denote the moment of inertia matrix about the CG, expressed in the body frame. $\mathcal{F}_\mathrm{B}$. The aircraft dynamics are
$$
	\begin{aligned}
		\dot{\bm{v}} &= \bm{v}\times\bm{\omega} + \bm{R}_\mathrm{IB}^\mathsf{T}\bm{g} + \frac{1}{m}\bm{F} \\ 
		\dot{\bm{\omega}} &= \bm{I}^{-1}( \bm{I}\bm{\omega} \times \bm{\omega} + \bm{M} )
	\end{aligned}
$$

Defining the generalized mass matrix $\mathscr{M} = \mathbf{diag}(m\mathbb{I}, \bm{I})$ and the generalized force $\mathscr{F} = [\bm{F}^\mathsf{T}~\bm{M}^\mathsf{T}]^\mathsf{T}$, the aircraft dynamics are written compactly as
$$
	\dot{\bm{\nu}} = \bm{f}_\nu(\bm{\eta},\bm{\nu}) + \mathscr{M}^{-1}\mathscr{F} 
$$

Altogether, the state of the rigid body is 
$$
	\bm{x}_\mathrm{rb} = \begin{bmatrix} 
		\bm{\eta} \\
		\bm{\nu}
	\end{bmatrix} \in \mathbb{R}^{n_\mathrm{rb}}
$$
where $n_\mathrm{rb} = n_\mathrm{pose} + 9$.