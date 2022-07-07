# MRAC (Model Reference Adaptive Control)
 Considering the LTI system

$$
\dot{x}=Ax+Bu\\
y=Cx
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
\dot{x}_m=A_mx_m+B_mr\\
y=Cx_m
$$

* $r\in\mathbb{R}^p$: reference vector

This model complies that:

* $A_m=A+BK_x^*$
* $B_m=BK_r^*$

### Observer
Since not all states are accessible, a Luenberger observer is created as:
$$
\dot{\hat{x}}=A\hat{x}+Bu+L(y-\hat{y})\\
\hat{y}=C\hat{x}
$$
$$
\dot{\hat{x}}=A\hat{x}+Bu+L(y-C\hat{x})\\
\dot{\hat{x}}=A\hat{x}+Bu+Ly-LC\hat{x}\\
\dot{\hat{x}}=(A-LC)\hat{x}+Bu+Ly
$$

* $BK_y^*=-L$
Observer error:
$$
e_l=x-\hat{x}\\
\dot{e}_l=\dot{x}-\dot{\hat{x}}\\
\dot{e}_l=Ax+Bu-(A-LC)\hat{x}-Bu-Ly\\
\dot{e}_l=Ax-(A-LC)\hat{x}-LCx\\
\dot{e}_l=(A-LC)x-(A-LC)\hat{x}\\
\dot{e}_l=(A-LC)e_l
$$

### Controller
Using the next control law:
$$
u=K_x^*\hat{x}+K_r^*r+K_y^*(y-\hat{y})
$$

The state estimate is:
$$
\dot{\hat{x}}=(A_m-BK_x^*)\hat{x}+B(K_x\hat{x}+K_rr+K_y(y-\hat{y}))+L(y-\hat{y})
$$


### Error
Defining the error as:
$$
e=x_m-\hat{x}\\
\dot{e}=\dot{x}_m-\dot{\hat{x}}\\
\dot{e}=A_mx_m+B_mr-A_m\hat{x}+BK_x^*\hat{x}-BK_x\hat{x}-BK_rr-BK_y(y-\hat{y})-L(y-\hat{y})\\
\dot{e}=A_me+BK_r^*r+BK_x^*\hat{x}-BK_x\hat{x}-BK_rr-BK_y(y-\hat{y})+BK_y^*(y-\hat{y})
$$

Defining:
* $\tilde{K}_x = K_x-K_x^*$
* $\tilde{K}_r = K_r-K_r^*$
* $\tilde{K}_y = K_y-K_y^*$

$$
\dot{e}=A_me-B\tilde{K}_rr-B\tilde{K}_x\hat{x}-B\tilde{K}_y(y-\hat{y})\\
\dot{e}=A_me-B\tilde{K}_rr-B\tilde{K}_x\hat{x}-B\tilde{K}_yCe_l\\
\dot{e}=A_me-B\left(\tilde{K}_rr+\tilde{K}_x\hat{x}+\tilde{K}_yCe_l\right)
$$


To find the $K$, we choice the following Lyapunov candidate:
$$
V = e^TPe + \text{trace}(\tilde{K}_x\Gamma_x^{-1}\tilde{K}^T_x) + \text{trace}(\tilde{K}_r\Gamma_r^{-1}\tilde{K}^T_r) + \text{trace}(\tilde{K}_y\Gamma_y^{-1}\tilde{K}^T_y) \\
\dot{V} = e^TP\dot{e} + \dot{e}^TPe + 2\text{trace}(\tilde{K}_x\Gamma_x^{-1}\dot{\tilde{K}}^T_x)+2\text{trace}(\tilde{K}_r\Gamma_r^{-1}\dot{\tilde{K}}^T_r)+2\text{trace}(\tilde{K}_y\Gamma_y^{-1}\dot{\tilde{K}}^T_y) \\
\dot{V} = e^TP(A_me-BK)+(A_me-BK)^TPe+ 2\text{trace}(\tilde{K}_x\Gamma_x^{-1}\dot{\tilde{K}}^T_x)+2\text{trace}(\tilde{K}_r\Gamma_r^{-1}\dot{\tilde{K}}^T_r)+2\text{trace}(\tilde{K}_y\Gamma_y^{-1}\dot{\tilde{K}}^T_y)\\
\dot{V} = e^TP(A_me-BK)+(e^TA_m^T-K^TB^T)Pe+ 2\text{trace}(\tilde{K}_x\Gamma_x^{-1}\dot{\tilde{K}}^T_x)+2\text{trace}(\tilde{K}_r\Gamma_r^{-1}\dot{\tilde{K}}^T_r)+2\text{trace}(\tilde{K}_y\Gamma_y^{-1}\dot{\tilde{K}}^T_y)\\
\dot{V} = e^TPA_me-e^TPBK+e^TA_m^TPe-K^TB^TPe+ 2\text{trace}(\tilde{K}_x\Gamma_x^{-1}\dot{\tilde{K}}^T_x)+2\text{trace}(\tilde{K}_r\Gamma_r^{-1}\dot{\tilde{K}}^T_r)+2\text{trace}(\tilde{K}_y\Gamma_y^{-1}\dot{\tilde{K}}^T_y)\\
\dot{V} = e^T[PA_me+e^TA_m^TP]e-e^TPBK-K^TB^TPe+ 2\text{trace}(\tilde{K}_x\Gamma_x^{-1}\dot{\tilde{K}}^T_x)+2\text{trace}(\tilde{K}_r\Gamma_r^{-1}\dot{\tilde{K}}^T_r)+2\text{trace}(\tilde{K}_y\Gamma_y^{-1}\dot{\tilde{K}}^T_y)\\
\dot{V} = e^T[PA_me+e^TA_m^TP]e-2e^TPB\left(\tilde{K}_rr+\tilde{K}_x\hat{x}+\tilde{K}_yCe_l\right)+ 2\text{trace}(\tilde{K}_x\Gamma_x^{-1}\dot{\tilde{K}}^T_x)+2\text{trace}(\tilde{K}_r\Gamma_r^{-1}\dot{\tilde{K}}^T_r)+2\text{trace}(\tilde{K}_y\Gamma_y^{-1}\dot{\tilde{K}}^T_y)
$$

* $e^TPB\tilde{K}_rr = \text{trace}(\tilde{K}_rre^TPB)$
* $e^TPB\tilde{K}_x\hat{x} = \text{trace}(\tilde{K}_x\hat{x}e^TPB)$
* $e^TPB\tilde{K}_yCe_l = \text{trace}(\tilde{K}_yCe_le^TPB)$

$$
\dot{V} = e^T[PA_me+e^TA_m^TP]e 
+ 2\text{trace}\left(\tilde{K}_x\left[-\hat{x}e^TPB+\Gamma_x^{-1}\dot{\tilde{K}}^T_x\right]\right)
+ 2\text{trace}\left(\tilde{K}_r\left[-re^TPB+\Gamma_r^{-1}\dot{\tilde{K}}^T_r\right]\right)
+ 2\text{trace}\left(\tilde{K}_y\left[-Ce_le^TPB+\Gamma_y^{-1}\dot{\tilde{K}}^T_y\right]\right)
$$

$$
\tilde{K}_i = K_i-K_i^*\\
\dot{\tilde{K}_i}=\dot{K}_i-\dot{K}_i^*\\
\dot{\tilde{K}_i}=\dot{K}_i
$$

* $\dot{K}^T_x = \Gamma_x\hat{x}e^TPB$
* $\dot{K}^T_r = \Gamma_rre^TPB$
* $\dot{K}^T_y = \Gamma_yCe_le^TPB$


$$ 

$$