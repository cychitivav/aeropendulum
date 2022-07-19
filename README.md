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
\begin{gather*}
    I_b = I_{b_{cm}} + m_bl_b^2\\
    I_b = \frac{1}{12}m_b(l_1+l_m)^2 + m_bl_b^2
\end{gather*}
$$

And the motor+nuts inertia ($I_n$) is considered as a puntual load, i.e.

$$
I_n = m_nl_n^2
$$

## State-space model
To represent the state-space model of the system, we define the next state equations:

$$\begin{gather*}
    x_1 = \theta\\
    x_2 = \dot{\theta}
\end{gather*}$$

Therefore:

$$
\begin{gather*}
    \dot{x}_1 = x_2\\
    \dot{x}_2 = \frac{1}{I}(W\cos{x_1}-\beta x_2+F_hl_m)
\end{gather*}
$$

### Linearization
To design linear controllers, we need to linearize the system around the an equilibrium point.

$$
\begin{gather*}
    f_1(x_1,x_2,u) = \dot{x}_1\\
    f_2(x_1,x_2,u) = \dot{x}_2
\end{gather*}
$$

$$
\begin{gather*}
    X^\ast=
    \begin{bmatrix}
        x_1^\ast\\
        x_2^\ast
    \end{bmatrix}\\
    U^\ast=F^\ast\\
    Z = \Delta X = X-X^\ast
\end{gather*}
$$

$$
\begin{gather*}
    \dot{Z}=\left.
    \begin{bmatrix}
    \frac{\partial f_1}{\partial x_1} & \frac{\partial f_1}{\partial x_2}\\
        \frac{\partial f_2}{\partial x_1} & \frac{\partial f_1}{\partial x_2} 
    \end{bmatrix}\right|\_{X^\ast , F^\ast}
    Z+\left.
    \begin{bmatrix}
        \frac{\partial f_1}{\partial u}\\
        \frac{\partial f_2}{\partial u} 
    \end{bmatrix}\right|_{X^\ast,F^\ast}
    u
\end{gather*}
$$


##### Equilibrium points
For $x_2^\ast$:

$$
\begin{gather*}
    f_1(x_1^\ast,x_2^\ast,F^\ast)=\dot{x}_1=0\\
    x_2^\ast=0
\end{gather*}
$$

To $x_1$:

$$
\begin{gather*}
    f_2(x_1\ast,x_2^\ast,F^\ast)=\dot{x}_2=0\\
    \frac{1}{I}(W\cos{x_1^\ast}-\beta x_2^\ast+F^\ast l_m)=0\\
    W\cos{x_1^\ast}-\beta x_2^\ast+F^\ast l_m=0\\
    F^\ast=\frac{\beta x_2^\ast-W\cos{x_1^\ast}}{l_m}\\
    \mathbf{F^\ast=-\frac{W\cos{x_1^\ast}}{l_m}}
\end{gather*}
$$ 



Finally:

$$
\begin{gather*}
    \dot{Z}=\left.
    \begin{bmatrix}
        0 & 1\\
        -\frac{W}{I}\sin{x_1} & -\frac{\beta}{I}
    \end{bmatrix}\right|\_{X^\ast,F^\ast}
    Z+\left.
    \begin{bmatrix}
        0\\
        \frac{l_m}{I} 
    \end{bmatrix}\right|_{X^\ast,F^\ast}
    u\\
    \dot{Z}=
    \begin{bmatrix}
        0 & 1\\
        -\frac{W}{I}\sin{x_1^\ast} & -\frac{\beta}{I}
    \end{bmatrix}
    Z+
    \begin{bmatrix}
        0\\
        \frac{l_m}{I} 
    \end{bmatrix}
    u
\end{gather*}
$$

If you want $x_1^\ast=0$, the thrust force is $F^\ast=-\frac{W}{l_m}$ and:

$$
\begin{gather*}
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
\end{gather*}
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
    \begin{gather*}
        B=
        \begin{bmatrix}
        0\\
        K_f\frac{l_m}{I}
        \end{bmatrix}
    \end{gather*}
    $$

# SMC   
Define the next states:

$$
\begin{gather*}
    \dot{x}_1 = x_2\\
    \dot{x}_2 = \frac{1}{I}(W\cos{x_1}-\beta x_2+F_hl_m)
\end{gather*}
$$

Can be written as:

$$
\begin{gather*}
    \dot{x}_1 = x_2\\
    \dot{x}_2 = \frac{1}{I}[W\cos{x_1}-\beta x_2+(K_f\cdot v+b_f)l_m]
\end{gather*}
$$

* $h(x) = \frac{1}{I}(W\cos{x_1}-\beta x_2+b_fl_m)$
* $f(x) = \frac{1}{I}(K_fl_m)$

Now, we define an error:

$$
\begin{gather*}
    x_e=x_1-x_r\\
    \dot{x}_e=\dot{x}_1
\end{gather*}
$$

and the next sliding manifold:

$$
S = ax_e+x_2
$$

where its derivative is:

$$
\begin{gather*}
    \dot{S} = a\dot{x}_e+\dot{x}_2\\
    \dot{S} = a\dot{x}_1+h(x)+f(x)v
\end{gather*}
$$

and the signal control law is:

$$
v=-\rho\ \text{sign}(S)
$$

To the system stability, we need to define the following:

$$
\begin{gather*}
    \rho \geq \left|\frac{ax_2+h(x)}{f(x)}\right|\\
    \rho \geq \left|\frac{ax_2+\frac{1}{I}(W\cos{x_1}-\beta x_2+b_fl_m)}{\frac{1}{I}(K_fl_m)}\right|
\end{gather*}
$$

Taken as limits $x_2\leq\pi$ and, we get:

$$
\begin{gather*}
    \rho \geq \frac{a\max{x_2}+\frac{1}{\min{I}}(\max{W}+\max{\beta}\max{x_2}+\max{b_fl_m})}{\frac{1}{\max{I}}\min{K_fl_m}}\\
    \rho \geq \frac{a\pi+\frac{1}{I_0}(\max{W}-\max{\beta}\pi+\max{b_fl_m})}{\frac{1}{I}\min{K_fl_m}}
\end{gather*}
$$

# MRAC (Model Reference Adaptive Control)
 Considering the LTI system

$$
\begin{gather*}
    \dot{x}=Ax+Bu\\
    y=Cx
\end{gather*}
$$

where:
* $x\in	\mathbb{R}^n$: state vector
* $u\in	\mathbb{R}^m$: control input vector 
* $y\in	\mathbb{R}^p$: output vector
* $A\in\mathbb{R}^{n\times n}$: state matrix (Unkonwn)
* $B\in\mathbb{R}^{n\times m}$: input matrix (Partially known)
* $C\in\mathbb{R}^{p\times n}$: output matrix (Known)



Now, we define a reference model such that in open loop the interesting signal follows the reference:

$$
\begin{gather*}
    \dot{x}_m=A_mx_m+B_mr\\
    y=Cx_m
\end{gather*}
$$

* $r\in\mathbb{R}^p$: reference vector

This model complies that:

* $A_m=A+BK_x^\ast$
* $B_m=BK_r^\ast$

### Observer
Since not all states are accessible, a Luenberger observer is created as:

$$
\begin{gather*}
    \dot{\hat{x}}=A\hat{x}+Bu+L(y-\hat{y})\\
    \hat{y}=C\hat{x}
\end{gather*}
$$

$$
\begin{gather*}
    \dot{\hat{x}}=A\hat{x}+Bu+L(y-C\hat{x})\\
    \dot{\hat{x}}=A\hat{x}+Bu+Ly-LC\hat{x}\\
    \dot{\hat{x}}=(A-LC)\hat{x}+Bu+Ly
\end{gather*}
$$

* $BK_y^\ast=-L$

Observer error:

$$
\begin{gather*}
    e_l=x-\hat{x}\\
    \dot{e}_l=\dot{x}-\dot{\hat{x}}\\
    \dot{e}_l=Ax+Bu-(A-LC)\hat{x}-Bu-Ly\\
    \dot{e}_l=Ax-(A-LC)\hat{x}-LCx\\
    \dot{e}_l=(A-LC)x-(A-LC)\hat{x}\\
    \dot{e}_l=(A-LC)e_l
\end{gather*}
$$

### Controller
Using the next control law:

$$
u=K_x^\ast\hat{x}+K_r^\ast r+K_y^\ast(y-\hat{y})
$$

The state estimate is:

$$
\dot{\hat{x}}=(A_m-BK_x^\ast)\hat{x}+B(K_x\hat{x}+K_rr+K_y(y-\hat{y}))+L(y-\hat{y})
$$


### Error
Defining the error as:

$$
\begin{gather*}
    e=x_m-\hat{x}\\
    \dot{e}=\dot{x}_m-\dot{\hat{x}}\\
    \dot{e}=A_mx_m+B_mr-A_m\hat{x}+BK_x^\ast\hat{x}-BK_x\hat{x}-BK_rr-BK_y(y-\hat{y})-L(y-\hat{y})\\
    \dot{e}=A_me+BK_r^\ast r+BK_x^\ast\hat{x}-BK_x\hat{x}-BK_rr-BK_y(y-\hat{y})+BK_y^\ast(y-\hat{y})
\end{gather*}
$$

Defining:
* $\tilde{K}_x = K_x-K_x^\ast$
* $\tilde{K}_r = K_r-K_r^\ast$
* $\tilde{K}_y = K_y-K_y^\ast$

$$
\begin{gather*}
    \dot{e}=A_me-B\tilde{K}_rr-B\tilde{K}_x\hat{x}-B\tilde{K}_y(y-\hat{y})\\
    \dot{e}=A_me-B\tilde{K}_rr-B\tilde{K}_x\hat{x}-B\tilde{K}_yCe_l\\
    \dot{e}=A_me-B\left(\tilde{K}_rr+\tilde{K}_x\hat{x}+\tilde{K}_yCe_l\right)
\end{gather*}
$$


To find the $K$, we choice the following Lyapunov candidate:

$$
\begin{gather*}
    V = e^TPe + \text{trace}(\tilde{K}_x\Gamma_x^{-1}\tilde{K}^T_x) + \text{trace}(\tilde{K}_r\Gamma_r^{-1}\tilde{K}^T_r) + \text{trace}(\tilde{K}_y\Gamma_y^{-1}\tilde{K}^T_y) \\
    \dot{V} = e^TP\dot{e} + \dot{e}^TPe + 2\text{trace}(\tilde{K}_x\Gamma_x^{-1}\dot{\tilde{K}}^T_x)+2\text{trace}(\tilde{K}_r\Gamma_r^{-1}\dot{\tilde{K}}^T_r)+2\text{trace}(\tilde{K}_y\Gamma_y^{-1}\dot{\tilde{K}}^T_y) \\
    \dot{V} = e^TP(A_me-BK)+(A_me-BK)^TPe+ 2\text{trace}(\tilde{K}_x\Gamma_x^{-1}\dot{\tilde{K}}^T_x)+2\text{trace}(\tilde{K}_r\Gamma_r^{-1}\dot{\tilde{K}}^T_r)+2\text{trace}(\tilde{K}_y\Gamma_y^{-1}\dot{\tilde{K}}^T_y)\\
    \dot{V} = e^TP(A_me-BK)+(e^TA_m^T-K^TB^T)Pe+ 2\text{trace}(\tilde{K}_x\Gamma_x^{-1}\dot{\tilde{K}}^T_x)+2\text{trace}(\tilde{K}_r\Gamma_r^{-1}\dot{\tilde{K}}^T_r)+2\text{trace}(\tilde{K}_y\Gamma_y^{-1}\dot{\tilde{K}}^T_y)\\
    \dot{V} = e^TPA_me-e^TPBK+e^TA_m^TPe-K^TB^TPe+ 2\text{trace}(\tilde{K}_x\Gamma_x^{-1}\dot{\tilde{K}}^T_x)+2\text{trace}(\tilde{K}_r\Gamma_r^{-1}\dot{\tilde{K}}^T_r)+2\text{trace}(\tilde{K}_y\Gamma_y^{-1}\dot{\tilde{K}}^T_y)\\
    \dot{V} = e^T[PA_me+e^TA_m^TP]e-e^TPBK-K^TB^TPe+ 2\text{trace}(\tilde{K}_x\Gamma_x^{-1}\dot{\tilde{K}}^T_x)+2\text{trace}(\tilde{K}_r\Gamma_r^{-1}\dot{\tilde{K}}^T_r)+2\text{trace}(\tilde{K}_y\Gamma_y^{-1}\dot{\tilde{K}}^T_y)\\
    \dot{V} = e^T[PA_me+e^TA_m^TP]e-2e^TPB\left(\tilde{K}_rr+\tilde{K}_x\hat{x}+\tilde{K}_yCe_l\right)+ 2\text{trace}(\tilde{K}_x\Gamma_x^{-1}\dot{\tilde{K}}^T_x)+2\text{trace}(\tilde{K}_r\Gamma_r^{-1}\dot{\tilde{K}}^T_r)+2\text{trace}(\tilde{K}_y\Gamma_y^{-1}\dot{\tilde{K}}^T_y)
\end{gather*}
$$

* $e^TPB\tilde{K}_rr = \text{trace}(\tilde{K}_rre^TPB)$
* $e^TPB\tilde{K}_x\hat{x} = \text{trace}(\tilde{K}_x\hat{x}e^TPB)$
* $e^TPB\tilde{K}_yCe_l = \text{trace}(\tilde{K}_yCe_le^TPB)$

$$
\begin{gather*}
    \dot{V} = e^T[PA_me+e^TA_m^TP]e 
    + 2\text{trace}\left(\tilde{K}_x\left[-\hat{x}e^TPB+\Gamma_x^{-1}\dot{\tilde{K}}^T_x\right]\right)
    + 2\text{trace}\left(\tilde{K}_r\left[-re^TPB+\Gamma_r^{-1}\dot{\tilde{K}}^T_r\right]\right)
    + 2\text{trace}\left(\tilde{K}_y\left[-Ce_le^TPB+\Gamma_y^{-1}\dot{\tilde{K}}^T_y\right]\right)
\end{gather*}
$$

$$
\begin{gather*}
    \tilde{K}_i = K_i-K_i^\ast\\
    \dot{\tilde{K}_i}=\dot{K}_i-\dot{K}_i^\ast\\
    \dot{\tilde{K}_i}=\dot{K}_i
\end{gather*}
$$

* $\dot{K}^T_x = \Gamma_x\hat{x}e^TPB$
* $\dot{K}^T_r = \Gamma_rre^TPB$
* $\dot{K}^T_y = \Gamma_yCe_le^TPB$





