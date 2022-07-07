# SMC   
DEfine the next states:
$$
\dot{x}_1 = x_2\\
\dot{x}_2 = \frac{1}{I}(W\cos{x_1}-\beta x_2+F_hl_m)
$$

Can be written as:
$$
\dot{x}_1 = x_2\\
\dot{x}_2 = \frac{1}{I}[W\cos{x_1}-\beta x_2+(K_f\cdot v+b_f)l_m]
$$

* $h(x) = \frac{1}{I}(W\cos{x_1}-\beta x_2+b_fl_m)$
* $f(x) = \frac{1}{I}(K_fl_m)$

Now, we define an error:

$$
x_e=x_1-x_r\\
\dot{x}_e=\dot{x}_1
$$

and the next sliding manifold:
$$
S = ax_e+x_2
$$

where its derivative is:
$$
\dot{S} = a\dot{x}_e+\dot{x}_2\\
\dot{S} = a\dot{x}_1+h(x)+f(x)v
$$

and the signal control law is:
$$
v=-\rho\ \text{sign}(S)
$$

To the system stability, we need to define the following:
$$
\rho \geq \left|\frac{ax_2+h(x)}{f(x)}\right|\\
\rho \geq \left|\frac{ax_2+\frac{1}{I}(W\cos{x_1}-\beta x_2+b_fl_m)}{\frac{1}{I}(K_fl_m)}\right|
$$

Taken as limits $x_2\leq\pi$ and, we get:

$$
\rho \geq \frac{a\max{x_2}+\frac{1}{\min{I}}(\max{W}+\max{\beta}\max{x_2}+\max{b_fl_m})}{\frac{1}{\max{I}}\min{K_fl_m}}\\
\rho \geq \frac{a\pi+\frac{1}{I_0}(\max{W}-\max{\beta}\pi+\max{b_fl_m})}{\frac{1}{I}\min{K_fl_m}}
$$