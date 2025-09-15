## Lecture 3 - Frames
### Notation
---
First we will introduce Monogram notation. Suppose we are talking about a point in 3D space, call this A. Call some other point B. Then

$^Bp^A$ is the position of A relative to B, a vector. A is called the target, B is called the reference.

A special coordinate frame is the world frame W. 
$^Wp^A=p^A$ is the position of A relative to the World origin.

When looking at a coordinate frame, x-axis will always be colored red, y-axis colored green, and z-axis colored blue. Now, given a coordinate frame,

$^Bp^A_C$ is the position of A relative to B, a vector in coordinate frame C. Written as P_BA_C in code.
$^Cp^A= ^Cp^A_C=$ is the position of A relative to the origin of coordinate frame C.

Let $R$ denote a rotation. Similar to position, we have

$^BR^A$ as the rotation of frame A relative to frame B.

Let $X$ denote a transform, which consists of a translation and rotation. Then

$^BX^A$ is the pose/transform of frame A relative to frame B. We can also express this as ($^Bp^A_B,^BR^A$)

Let's go through an example.
![[Pasted image 20250910161335.png]]
We are being asked for the position of the object relative to the gripper. The object is along the positive y-axis of the gripper's coordinate frame, so the only reasonable answer choice is (b).

![[Pasted image 20250910161818.png]]
We switch the coordinate from to the O frame. Note that we are looking at the same vector, just expressed in a different frame. Relative to the O frame, we have a negative z-component, so the only reasonable answer is (a).

### Spatial Algebra
---
We can add two positions in the same frame as follows:
$$^Ap^B_F+^Bp^C_F=^Ap^C_F$$
It is invalid to add vectors from different frames. 

Addition is commutative, and its additive inverse is well defined:
$$
^Ap^B_F=-^Bp^A_F
$$
Multiplication by a rotation can be used to change the "expressed in" frame. 
$$
^Ap^B_G=(^GR^F)(^Ap^B_F)
$$
Composing rotations together is valid when their reference and target symbols match.
$$
^AR^C=(^AR^B)(^BR^C)
$$
The inverse of multiplication is also well defined.
$$
[^AR^B]^{-1}=[^BR^A]
$$
Finally, transforms compose
$$
[^AX^B][^BX^C]=[^AX^C]
$$
and also have an inverse
$$
[^AX^B]^{-1}=^BX^A
$$

### Rotations
---
#### Rotation Matrix
In 3D, rotations can be expressed via a 3x3 rotation matrix $R$. It has a few properties
* R is orthonormal, which implies that
$$
R^TR=I=RR^T
$$
and
$$
\det(R)=+1
$$
* The columns are the x, y, z axes of the target frame relative to the reference. To make this clear, if $A$ is the target and $B$ is the reference, then the first column of $^GR^F$ is the x-axis of $A$ expressed in $B$ and so on. Moreover, the rows are the x, y, z axes of the reference frame relative to the target.
#### Euler Angles
Often used in URDF files, Euler angles are a set of three numbers: roll, pitch, and yaw. 

To remember the sign of rotation, think about a cyclical arrangement of xyz. 
* Going from x to y, a rotation in the z-axis is necessary. Going from y to z, a rotation in the x-axis is necessary. Going from z to x, a rotation in the y-axis is necessary. A visual interpretation is given below:
$$
+x\stackrel{z}{\rightarrow}+y\stackrel{x}{\rightarrow}+z\stackrel{y}{\rightarrow}+x
$$

The direction of the arrows represents the direction of a positive rotation. 

Note that composing rotation matrices is order-dependent (i.e. $XY\neq YX$). This is a problem because when we talk about rotating about an axis, are we talking about the original axis or the new one? There is the extrinsic convention in which we are rotating about the original xyz axes and the intrinsic convention that we are rotating about the new frame we ended up with. This is the problem with euler angles.

Roll, pitch, and yaw use the extrinsic convention and is commonly used in URDF files. 

An issue with the rotation matrix is that it has many numbers to represent and has no singularities (also is unique), an issue with the euler angles is that it only has three numbers with some rotation but some rotations have infinite representations. 
#### Axis Angle Representation
Also defined by three numbers
$$
a=\begin{bmatrix}
x \\
y\\
z\\
\end{bmatrix}
$$
where 
$$
\hat{a}=\frac{a}{\left\lVert a\right\rVert}
$$
is the axis of rotation and $\left\lVert a\right\rVert$ is the magnitude (angle) of rotation. 

A singularity here is that when the angle starts getting small, then we start losing precision and this representation starts misbehaving. At 0, our rotation is undefined. It is also not unique (we can add $2\pi$ to our angles)

It is not possible to have no singularities with only three numbers
#### Quaternions
A quaternion is represented by
$$
q=\begin{bmatrix}
q_w & q_x & q_y & q_z
\end{bmatrix}
$$
which we note has four numbers. We can interpret it as $q_w$, which is a constant appended to a vector $\begin{bmatrix}q_x & q_y & q_z\end{bmatrix}$. To represent a rotation, we want the entire quaternion to be of unit magnitude. 

This is a nice, simple way of composing rotations, but it is not nice to apply to a vector. You first have to build a matrix to apply it to a vector. 

The inverse quaternion is 
$$
\begin{bmatrix}
q_w & -q_x & -q_y & -q_z
\end{bmatrix}
$$
Quaternions don't have a singularity, but it is still nonunique. You can take the "negative axis" and "negative angle" and yield the same quaternion. In other words, 
$$
q=-q
$$
A quaternion can be thought of a sphere in 4-dimensional space. The top half parametrizes rotations and the bottom half parametrizes rotation, so the negative points are the same rotation. 
### Transforms
---
We use a 4x4 matrix to represent a transform.
$$
X=\begin{bmatrix}
& & & \mid  \\
& R & & \mid & d\\
& & & \vert\\
\hline
0 & 0 & 0 &\vert & 1
\end{bmatrix}
$$
where R is a 3x3 rotation matrix and d is a 3x1 displacement vector. This is also known as a homogeneous transformation. When we compose it with a vector, we have
$$
Xv=\begin{bmatrix}
& & & \mid  \\
& R & & \mid & d\\
& & & \vert\\
\hline
0 & 0 & 0 &\vert & 1
\end{bmatrix}\begin{bmatrix}
x\\ y\\ z\\1
\end{bmatrix}
$$
When we multiply this through, we notice that the transformation rotates $v$ and adds $d$, thus making it a proper transform. 

## Lecture 4 - Kinematics

### Inverse Kinematics
---
We want to go from pose $\rightarrow$ joint positions
$$
X^B\to q$$
Difficult in general, may be no solution, multiple solutions, or infinite solutions by solving trigonometric equations

Analytical Solution - Can use URDF files IKFAST to get analytic solution
Numerical Solution - Given initial guess at configuration, do e.g. numerical root finding

iiwa robot has seven degrees of freedom, so there are infinitely many solutions to the inverse kinematics

### Differential Kinematics
We want to go from joint positions, velocities $\to$ spatial velocity
$$
q,v\to V^B$$
We have 
$$
X^B = f^B_{kin}(q)
$$
which implies that
$$
dX^B=\frac{df^B_{kin}(q)}{dq}dq=J^B(q)dq
$$
where $J^B(q)$ is the kinematic Jacobian. The $B$ superscript indicates which frame we are in.

### Spatial Velocity
The spatial velocity is defined by an angular velocity $^A\omega^B_C$ and a translational velocity $^Av^B_C$.
$$
\frac{d}{dt}[^AX^B_C]=[^AV^B_C]=\begin{bmatrix}
^A\omega^B_C\\
^Av^B_C
\end{bmatrix}
$$
Just like with spatial algebra, we also have spatial velocity algebra. 

Rotations can be used to change between the "expressed-in" frames.
$$
^Av^B_G=[^GR^F][^Av^B_F]
$$
$$
^A\omega^B_G=[^GR^F][^A\omega^B_F]
$$
Angular velocities add when they are expressed in the same frame
$$
^A\omega^B_F+^B\omega^C_F=^A\omega^C_F
$$
We also have the additive inverse
$$
^A\omega^C_F=-^C\omega^A_F
$$
Composition of translational velocities is 
$$
TODO ADD THE REST
$$

### Jacobian
Analytic Jacobians relate joint velocities to derivatives of pose parameters.

Geometric Jacobian relates joint velocities to spatial velocity.

The angular velocity and the derivative of orientation parameters are related by a matrix: $N(q)$, 

In other words, we have
$$
\dot{q}=N(q)v
$$
where $v$ is the velocity is and $\dot{q}$ is the derivative of position. $N(q)$ is used for the rotational mapping between analytic and geometric Jacobians which is usually composed as follows:
$$
\begin{bmatrix}
& & & \mid  \\
& N(q) & & \mid & 0\\
& & & \vert\\
\hline
 & 0 &  &\vert & I
\end{bmatrix}
$$
The analytic Jacobian may have a different size than the geometric Jacobian and may have difference reference points. 

$\textbf{Note:}$ We are interested in the inverse of the Jacobian, but this matrix may be nonsquare or singular, so it may not always have an inverse. Thus, we employ a "pseudo-inverse." If the matrix is full rank (aka we have an inverse), then we get a unique answer (the true inverse). If we have too few degrees of freedom (under full rank), then we get the least squares solution, the velocity that gets us "near" the one that we want. If we have too many degrees of freedom (redundant), now there are infinitely many joint velocities that produce a given joint velocity which leads us to the minimal norm solution (smallest quadratic norm solution).