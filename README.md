# Aeropendulum
This repository contains the development of an aeropendulum and the performance of various controllers to achieve angular positions between $30$ and $-30 \degree$.

<div align="center">
    <img src="https://user-images.githubusercontent.com/30636259/176775784-f84b0b3c-6dd6-4525-a0b1-dddd1b19d21e.png#gh-dark-mode-only" width="500" />
    <img src="https://user-images.githubusercontent.com/30636259/176775798-b19df07f-7b4e-49f5-8b4e-077d998794a8.png#gh-light-mode-only" width="500" />
</div>

$$
\begin{gather*}
    I\ddot{\theta}=W_cl_c\cos{\theta}+F_hl_m-W_bl_b\cos{\theta}-W_nl_n\cos{\theta}-B\dot{\theta}\\
    \ddot{\theta}=\frac{1}{I}(W\cos{\theta}-B\dot{\theta})+F_hl_m
\end{gather*}
$$

Where
$$
W=W_cl_c-W_bl_b-W_nl_n
$$

To simplify the inertia, we define the term $I_0$ as the system inertia without counterweight and the term $I$ as the system inertia with counterweight.

$$
I=I_0+\frac{W_c}{g}l_c^2\\
I_0 = I_b+I_n
$$

To compute the bar inertia ($I_b$), we use the parallel axes theorem:

$$
I_b = I_{b_{cm}} + m_bl_b^2\\
I_b = \frac{1}{12}m_b(l_1+l_m)^2 + m_bl_b^2\\
I_b = \frac{1}{12} \frac{W_b}{g} (l_1 + l_m)^2 + \frac{W_b}{g}l_b^2
$$

And the motor+nuts inertia ($I_n$) is considered as a puntual load, i.e.

$$
I_n = m_nl_n^2\\
I_n = \frac{W_n}{g} l_n^2
$$

#### State-space model
To represent the state-space model of the system, we define the next state equations:

$$
x_1 = \theta\\
x_2 = \dot{\theta}
$$

Therefore:

$$
\dot{x}_1 = x_2\\
\dot{x}_2 = \frac{1}{I}(W\cos{\theta}-B\dot{\theta})
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


### Linearization
To design linear controllers, we need to linearize the system around the desired angular position. 

## MRAC(Model Reference Apaptive Control)
