# Aeropendulum
This repository contains the development of an aeropendulum and the performance of various controllers to achieve angular positions between $30$ and $-30 \degree$.

<div align="center">
    <img src="https://user-images.githubusercontent.com/30636259/176775784-f84b0b3c-6dd6-4525-a0b1-dddd1b19d21e.png#gh-dark-mode-only" width="500" />
    <img src="https://user-images.githubusercontent.com/30636259/176775798-b19df07f-7b4e-49f5-8b4e-077d998794a8.png#gh-light-mode-only" width="500" />
</div>

$$
\begin{gather*}
    I\ddot{\theta}=W_cl_c\cos{\theta}+F_hl_m-W_bl_b\cos{\theta}-W_nl_n\cos{\theta}-\beta\dot{\theta}\\
    \ddot{\theta}=\frac{1}{I}(W\cos{\theta}-\beta\dot{\theta}+F_hl_m)
\end{gather*}
$$

Where
$$
W=W_cl_c-W_bl_b-W_nl_n
$$

To simplify the inertia, we define the term $I_0$ as the system inertia without counterweight and the term $I$ as the system inertia with counterweight.
$$
I=I_0+m_cl_c^2
$$

Where $I_0 = I_b+I_n$, to compute the bar inertia ($I_b$), we use the parallel axes theorem:
$$
I_b = I_{b_{cm}} + m_bl_b^2\\
I_b = \frac{1}{12}m_b(l_1+l_m)^2 + m_bl_b^2
$$

And the motor+nuts inertia ($I_n$) is considered as a puntual load, i.e.
$$
I_n = m_nl_n^2
$$

## State-space model
To represent the state-space model of the system, we define the next state equations:

$$
x_1 = \theta\\
x_2 = \dot{\theta}
$$

Therefore:

$$
\dot{x}_1 = x_2\\
\dot{x}_2 = \frac{1}{I}(W\cos{x_1}-\beta x_2+F_hl_m)
$$

### Linearization
To design linear controllers, we need to linearize the system around the an equilibrium point.

$$
f_1(x_1,x_2,u) = \dot{x}_1\\
f_2(x_1,x_2,u) = \dot{x}_2
$$

$$
X^*=
\begin{bmatrix}
    x_1^*\\
    x_2^*
\end{bmatrix}\\
U^*=F^*\\
Z = \Delta X = X-X^*
$$

$$
\dot{Z}=\left.
\begin{bmatrix}
\frac{\partial f_1}{\partial x_1} & \frac{\partial f_1}{\partial x_2}\\
    \frac{\partial f_2}{\partial x_1} & \frac{\partial f_1}{\partial x_2} 
\end{bmatrix}\right|_{X^*,F^*}
Z+\left.
\begin{bmatrix}
    \frac{\partial f_1}{\partial u}\\
    \frac{\partial f_2}{\partial u} 
\end{bmatrix}\right|_{X^*,F^*}
u
$$


##### Equilibrium points
For $x_2^*$:
$$
f_1(x_1^*,x_2^*,F^*)=\dot{x}_1=0\\
x_2^*=0
$$

To $x_1$:
$$
f_2(x_1^*,x_2^*,F^*)=\dot{x}_2=0\\
\frac{1}{I}(W\cos{x_1^*}-\beta x_2^*+F^*l_m)=0\\
W\cos{x_1^*}-\beta x_2^*+F^*l_m=0\\
F^*=\frac{\beta x_2^*-W\cos{x_1^*}}{l_m}\\
\mathbf{F^*=-\frac{W\cos{x_1^*}}{l_m}}
$$ 



Finally:
$$
\dot{Z}=\left.
\begin{bmatrix}
    0 & 1\\
    -\frac{W}{I}\sin{x_1} & -\frac{\beta}{I}
\end{bmatrix}\right|_{X^*,F^*}
Z+\left.
\begin{bmatrix}
    0\\
    \frac{l_m}{I} 
\end{bmatrix}\right|_{X^*,F^*}
u\\
\dot{Z}=
\begin{bmatrix}
    0 & 1\\
    -\frac{W}{I}\sin{x_1^*} & -\frac{\beta}{I}
\end{bmatrix}
Z+
\begin{bmatrix}
    0\\
    \frac{l_m}{I} 
\end{bmatrix}
u
$$

If you want $x_1^*=0$, the thrust force is $F^*=-\frac{W}{l_m}$ and:

$$
\dot{Z}=
\begin{bmatrix}
    0 & 1\\
    0 & -\frac{\beta}{I}
\end{bmatrix}
Z+
\begin{bmatrix}
    0\\
    \frac{l_m}{I} 
\end{bmatrix}
u
$$


### Characterization
To find the $B$ and $F_h$ parameters, we use the following methods:

1. **Damping coefficient ($B$)** <br>
    This parameter was found using the simple pendulum model. First, we remove the counterweight from the system and drop the bar with the motor from $0\degree$ to $30\degree$ and take the curve of angle in the time. Then, we compute the damping coefficient $B$ with the envelope of this signal.

    $$
    \text{env}(t) = ae^{-bt}
    $$

    $$
    B=2I_0b
    $$

1. **Thrust force ($F_h$)** <br>
    The objective of this identification is to find the thrust force $F_h$ that depends of voltage. Therefore, we take measurements of the voltage locating the system in an angle $0$ with different counterweights. With this, we can calculate the thrust force $F_h$ that corresponds to the voltage $V$ in the system making a torque summation.

    If we have:
    $$
    B=
    \begin{bmatrix}
    0\\
    K_f\frac{l_m}{I}
    \end{bmatrix}
    $$




