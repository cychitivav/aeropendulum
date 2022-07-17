# PID controller
$$
Z = \Delta X = X-X^*\\
X = Z+X^*\\

\dot{X} = \dot{Z}\\
\dot{X} = \left.
\begin{bmatrix}
\frac{\partial f_1}{\partial x_1} & \frac{\partial f_1}{\partial x_2}\\
    \frac{\partial f_2}{\partial x_1} & \frac{\partial f_1}{\partial x_2} 
\end{bmatrix}\right|_{X^*,F^*}
Z+\left.
\begin{bmatrix}
    \frac{\partial f_1}{\partial u}\\
    \frac{\partial f_2}{\partial u} 
\end{bmatrix}\right|_{X^*,F^*}
u\\

% \dot{X} = \left.
% \begin{bmatrix}
% \frac{\partial f_1}{\partial x_1} & \frac{\partial f_1}{\partial x_2}\\
%     \frac{\partial f_2}{\partial x_1} & \frac{\partial f_1}{\partial x_2} 
% \end{bmatrix}\right|_{X^*,F^*}
% (X-X^*)+\left.
% \begin{bmatrix}
%     \frac{\partial f_1}{\partial u}\\
%     \frac{\partial f_2}{\partial u} 
% \end{bmatrix}\right|_{X^*,F^*}
% u\\

% \dot{X} = \left.
% \begin{bmatrix}
%     \frac{\partial f_1}{\partial x_1} & \frac{\partial f_1}{\partial x_2}\\
%     \frac{\partial f_2}{\partial x_1} & \frac{\partial f_1}{\partial x_2} 
% \end{bmatrix}\right|_{X^*,F^*}
% X-
% \left.
% \begin{bmatrix}
%     \frac{\partial f_1}{\partial x_1} & \frac{\partial f_1}{\partial x_2}\\
%     \frac{\partial f_2}{\partial x_1} & \frac{\partial f_1}{\partial x_2} 
% \end{bmatrix}\right|_{X^*,F^*}X^* + \left.
% \begin{bmatrix}
%     \frac{\partial f_1}{\partial u}\\
%     \frac{\partial f_2}{\partial u} 
% \end{bmatrix}\right|_{X^*,F^*}
% u\\

% \dot{X} = \left.
% \begin{bmatrix}
% \frac{\partial f_1}{\partial x_1} & \frac{\partial f_1}{\partial x_2}\\
%     \frac{\partial f_2}{\partial x_1} & \frac{\partial f_1}{\partial x_2} 
% \end{bmatrix}\right|_{X^*,F^*}
% X + \left.
% \begin{bmatrix}
%     \frac{\partial f_1}{\partial u}\\
%     \frac{\partial f_2}{\partial u} 
% \end{bmatrix}\right|_{X^*,F^*}
% u-
% \left.
% \begin{bmatrix}
%     \frac{\partial f_1}{\partial x_1}x_1^* & \frac{\partial f_1}{\partial x_2}x_2^*\\
%     \frac{\partial f_2}{\partial x_1}x_1^* & \frac{\partial f_1}{\partial x_2} x_2^*
% \end{bmatrix}\right|_{X^*,F^*}
$$

<!-- 
$$
\dot{X}=
\begin{bmatrix}
    0 & 1\\
    -\frac{W}{I}\sin{x_1^*} & -\frac{\beta}{I}
\end{bmatrix}
X+
\begin{bmatrix}
    0\\
    \frac{l_m}{I} 
\end{bmatrix}
u
-
\begin{bmatrix}
    0 & x_2^*\\
    -\frac{W}{I}x_1^*\sin{x_1^*} & -\frac{\beta}{I}x_2^*
\end{bmatrix}
$$ -->