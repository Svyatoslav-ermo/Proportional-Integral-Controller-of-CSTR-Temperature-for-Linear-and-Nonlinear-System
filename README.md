# Proportional-Integral-Controller-of-CSTR-Temperature-for-Linear-and-Nonlinear-System
The project involves developing and analyzing PI controller for CSTR using MATLAB. Key tasks include solving nonlinear system, calculating the Jacobian matrix, performing eigenvalue analysis to assess system stability, and designing a PI controller. The controller's performance is optimized by tuning parameters for desired system behavior.

# Description

This MATLAB project involves process modeling, stability analysis, and controller design. The code is divided into several key sections:

# 1. Mass and Energy Balances
The project begins with the derivation of mass and energy balances for the process, which are detailed in the accompanying report.
# 2. Steady-State Analysis
Solves for steady-state conditions using the fsolve function to find equilibrium points for the process.
# 3. Jacobian Matrix and Stability
Constructs the Jacobian matrix of the system by symbolically differentiating the process equations. Eigenvalue analysis is then performed to evaluate the stability of each steady-state.
# 4. PI Controller Design
A Proportional-Integral (PI) controller is designed for the process. The transfer function of the plant is derived, and optimal controller parameters (Kc, tauI) are identified through iterative tuning for minimal settling time.
# 5. Controller Application
The designed PI controller is applied to both linear and nonlinear models of the system. The performance is validated through step response analysis.
# 6. Visualization
Step response plots are generated to visualize the system's behavior under the designed controller.
