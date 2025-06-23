# Linear Inverted Pendulum Stepping Controller

## Overview

This document presents a discrete stepping controller for bipedal robots based on the Linear Inverted Pendulum Model (LIPM). The controller determines optimal foot placement (step length) to achieve desired walking velocities while maintaining stability.

## 1. Linear Inverted Pendulum Model

### 1.1 Continuous-Time Dynamics

The linear inverted pendulum dynamics are governed by:

$$\ddot{x} = \frac{g}{h} x$$

Where:
- $x$ is the horizontal position of the center of mass (COM)
- $h$ is the height of the center of mass (assumed constant)
- $g$ is gravitational acceleration ($\approx 9.81$ m/s²)

### 1.2 State Space Representation

With state vector $\mathbf{s} = [x, \dot{x}]^T = [p, v]^T$, the continuous-time system becomes:

$$\dot{\mathbf{s}} = \begin{bmatrix} 0 & 1 \\ \frac{g}{h} & 0 \end{bmatrix} \mathbf{s}$$

### 1.3 Discrete-Time State Transition Matrix

For discrete-time analysis with time step $T$, the state transition matrix is:

$$\mathbf{A} = \begin{bmatrix} \cosh(\omega T) & \frac{1}{\omega}\sinh(\omega T) \\ \omega \sinh(\omega T) & \cosh(\omega T) \end{bmatrix}$$

Where $\omega = \sqrt{\frac{g}{h}}$ is the natural frequency of the inverted pendulum.

The discrete dynamics are:
$$\mathbf{s}(t=T) = \mathbf{A} \mathbf{s}(t=0)$$

## 2. Stepping Controller Design

### 2.1 Problem Setup

Let:
- $p_k(0)$, $v_k(0)$ = observed COM position and velocity at start of step $k$ (relative to stance foot)
- $p_k(T)$, $v_k(T)$ = COM position and velocity at end of step $k$
- $u$ = step length (distance between consecutive feet)
- $v_{des}$ = desired walking velocity

### 2.2 Kinematic Constraint

The relationship between consecutive foot placements and COM position is:

$$p_k(0) = u - p_{k-1}(T)$$

This constraint couples the foot placement decision with the COM dynamics.

### 2.3 Controller Derivation

At the start of step $k$, we predict the final state:

$$\begin{bmatrix} p_k(T) _{\text{pred}} \\ v_k(T) _{\text{pred}} \end{bmatrix} = \mathbf{A} \begin{bmatrix} p_k(0) \\ v_k(0) \end{bmatrix}$$

The stepping controller is designed to achieve $v_{k+1}(0) = v_{\text{des}}$ by setting:

$$u = p_k(T) _{\text{pred}} + \frac{v_{\text{des}} - A_{22} \cdot v_k(T) _{\text{pred}}}{A_{21}}$$

Where $A_{ij}$ denotes the $(i,j)$ element of matrix $\mathbf{A}$.

### 2.4 Expanded Controller Form

Substituting the matrix elements:

$$u = p_k(T) _{\text{pred}} + \frac{v_{\text{des}} - \cosh(\omega T) \cdot v_k(T) _{\text{pred}}}{\omega \sinh(\omega T)}$$

## 3. Stability Analysis

### 3.1 Step-to-Step Dynamics

The actual COM state at the end of step $k$ is:
$$\begin{bmatrix} p_k(T) \\ v_k(T) \end{bmatrix} = \mathbf{A} \begin{bmatrix} p_k(0) \\ v_k(0) \end{bmatrix}$$

The initial conditions for step $k+1$ are:
$$p_{k+1}(0) = u - p_k(T)$$
$$v_{k+1}(0) = v_k(T)$$

### 3.2 Closed-Loop Step-to-Step System

Substituting the controller into the step-to-step dynamics yields:

$$\begin{bmatrix} p_{k+1}(0) \\ v_{k+1}(0) \end{bmatrix} = \begin{bmatrix} -A_{22} & -\frac{A_{22}^2}{A_{21}} \\ A_{21} & A_{22} \end{bmatrix} \begin{bmatrix} p_k(0) \\ v_k(0) \end{bmatrix} + \begin{bmatrix} \frac{v_{\text{des}}}{A_{21}} \\ 0 \end{bmatrix}$$

### 3.3 Eigenvalue Analysis

The eigenvalues of the step-to-step matrix are:
$$\lambda_1 = A_{22} = \cosh(\omega T)$$
$$\lambda_2 = -A_{22} = -\cosh(\omega T)$$

### 3.4 Stability Conclusion

Since $\cosh(\omega T) > 1$ for any $T > 0$:
$$|\lambda_1| = |\lambda_2| = \cosh(\omega T) > 1$$

**The feedforward controller is inherently unstable.** Errors will grow exponentially between steps.

## 4. Feedback Stabilization

### 4.1 Stabilizing Feedback Controller

To achieve stability, add velocity and position feedback:

$$u = p_k(T) _{\text{pred}} + \frac{v_{\text{des}} - A_{22} \cdot v_k(T) _{\text{pred}}}{A_{21}} - K_p(v_k(0) - v_{\text{des}}) - K_d(p_k(0) - p_{\text{des}})$$

### 4.2 Minimum Stabilizing Gain

For velocity feedback alone, the minimum stabilizing gain is:

$$K_p > \frac{\cosh(\omega T) - 1}{\omega \sinh(\omega T)}$$

### 4.3 Deadbeat Control

Perfect velocity tracking in one step can be achieved by setting:

$$K_p = \frac{\cosh(\omega T) - 1}{\omega \sinh(\omega T)}$$

## 5. Implementation Considerations

### 5.1 Typical Parameter Values

For a humanoid robot with:
- Center of mass height: $h = 1.0$ m
- Time step: $T = 0.1$ s
- Natural frequency: $\omega = \sqrt{9.81/1.0} \approx 3.13$ rad/s

The state transition matrix becomes:
$$\mathbf{A} \approx \begin{bmatrix} 1.049 & 0.317 \\ 0.993 & 1.049 \end{bmatrix}$$

### 5.2 Practical Recommendations

1. **Always include velocity feedback** for step-to-step stability
2. **Add position feedback** for improved COM position control
3. **Include robustness margins** by choosing feedback gains 10-20% higher than minimum values
4. **Consider step length constraints** and saturation handling in practice
5. **Account for model uncertainties** and external disturbances

## 6. Enhanced Nonlinear Model

### 6.1 Quadratic Model Extension

To improve prediction accuracy, the linear model can be extended with quadratic terms that capture modeling errors:

$$\mathbf{s}(T) = \mathbf{A}\mathbf{s}(0) + \mathbf{f}(\mathbf{s}(0))$$

Where $\mathbf{f}(\mathbf{s})$ is a quadratic function:

$$\mathbf{f}(\mathbf{s}) = \begin{bmatrix} f_1(p,v) \\ f_2(p,v) \end{bmatrix} = \begin{bmatrix} c_1p^2 + c_2pv + c_3v^2 \\ c_4p^2 + c_5pv + c_6v^2 \end{bmatrix}$$

The coefficients $c_1$ through $c_6$ are typically identified from experimental data to capture nonlinear effects such as:
- Angular momentum variations
- Ground reaction force nonlinearities  
- Higher-order kinematic effects

### 6.2 Modified Controller Derivation

For the nonlinear model, the controller derivation starts from the desired velocity constraint:

$$v_{\text{des}} = A_{21} p_{k+1}(0) + A_{22} v_k(T) _{\text{pred}} + c_4 p_{k+1}(0)^2 + c_5 p_{k+1}(0) v_k(T) _{\text{pred}} + c_6 v_k(T) _{\text{pred}}^2$$

This is a **quadratic equation** in $p_{k+1}(0)$:

$$c_4 p_{k+1}(0)^2 + [A_{21} + c_5 v_k(T) _{\text{pred}}] p_{k+1}(0) + [A_{22} v_k(T) _{\text{pred}} + c_6 v_k(T) _{\text{pred}}^2 - v_{\text{des}}] = 0$$

### 6.3 Quadratic Solution for Optimal Position

The quadratic equation has the standard form $ap^2 + bp + c = 0$ where:
- $a = c_4$
- $b = A_{21} + c_5 v_k(T) _{\text{pred}}$  
- $c = A_{22} v_k(T) _{\text{pred}} + c_6 v_k(T) _{\text{pred}}^2 - v_{\text{des}}$

The solution is:
$$p_{k+1}(0) = \frac{-b \pm \sqrt{b^2 - 4ac}}{2a}$$

**Special Cases:**
- If $c_4 = 0$: Reduces to linear case $p_{k+1}(0) = \frac{v_{\text{des}} - A_{22}v_k(T) _{\text{pred}} - c_6 v_k(T) _{\text{pred}}^2}{A_{21} + c_5 v_k(T) _{\text{pred}}}$
- If $c_4 \neq 0$: Choose the root that gives a feasible step length

### 6.4 Complete Nonlinear Controller

The step length is then:
$$u = p_k(T) _{\text{pred}} + p_{k+1}(0)$$

Where the predicted final state includes both linear and nonlinear components:

$$\begin{bmatrix} p_k(T) _{\text{pred}} \\ v_k(T) _{\text{pred}} \end{bmatrix} = \mathbf{A} \begin{bmatrix} p_k(0) \\ v_k(0) \end{bmatrix} + \mathbf{f}\left(\begin{bmatrix} p_k(0) \\ v_k(0) \end{bmatrix}\right)$$

### 6.5 Nonlinear Stability Analysis

#### 6.5.1 Equilibrium Point Analysis

For steady-state walking, the equilibrium conditions become more complex due to the quadratic controller. At equilibrium:

$$p_{\text{eq}} = u_{\text{eq}} - [A_{11}p_{\text{eq}} + A_{12}v_{\text{des}} + f_1(p_{\text{eq}}, v_{\text{des}})]$$

And the desired velocity constraint gives:
$$v_{\text{des}} = A_{21}p_{\text{eq}} + A_{22}v_{\text{des}} + c_4 p_{\text{eq}}^2 + c_5 p_{\text{eq}} v_{\text{des}} + c_6 v_{\text{des}}^2$$

Rearranging the velocity equation:
$$v_{\text{des}}(1 - A_{22} - c_5 p_{\text{eq}}) = A_{21}p_{\text{eq}} + c_4 p_{\text{eq}}^2 + c_6 v_{\text{des}}^2$$

This creates a **coupled nonlinear system** that must be solved numerically for $(p_{\text{eq}}, u_{\text{eq}})$.

#### 6.5.2 Step-to-Step Dynamics

The step-to-step system becomes:
$$p_{k+1}(0) = u - p_k(T) = u - [A_{11}p_k(0) + A_{12}v_k(0) + f_1(p_k(0), v_k(0))]$$
$$v_{k+1}(0) = A_{21}p_k(0) + A_{22}v_k(0) + f_2(p_k(0), v_k(0))$$

Where $p_{k+1}(0)$ is the solution to the quadratic equation, making the system highly nonlinear.

#### 6.5.3 Linearization Around Equilibrium

Due to the implicit nature of the quadratic controller, the Jacobian becomes:

$$\mathbf{J} = \begin{bmatrix} \frac{\partial p_{k+1}(0)}{\partial p_k(0)} & \frac{\partial p_{k+1}(0)}{\partial v_k(0)} \\ \frac{\partial v_{k+1}(0)}{\partial p_k(0)} & \frac{\partial v_{k+1}(0)}{\partial v_k(0)} \end{bmatrix}\bigg|_{eq}$$

The Jacobian elements require implicit differentiation due to the quadratic constraint:

$$J_{11} = \frac{\partial p_{k+1}(0)}{\partial p_k(0)} = -[A_{11} + \frac{\partial f_1}{\partial p}] + \frac{\partial p_{k+1}(0)}{\partial v_k(T) _{\text{pred}}} \cdot [A_{21} + \frac{\partial f_2}{\partial p}]$$

$$J_{12} = \frac{\partial p_{k+1}(0)}{\partial v_k(0)} = -[A_{12} + \frac{\partial f_1}{\partial v}] + \frac{\partial p_{k+1}(0)}{\partial v_k(T) _{\text{pred}}} \cdot [A_{22} + \frac{\partial f_2}{\partial v}]$$

Where $\frac{\partial p_{k+1}(0)}{\partial v_k(T) _{\text{pred}}}$ comes from implicit differentiation of the quadratic constraint:

$$\frac{\partial p_{k+1}(0)}{\partial v_k(T) _{\text{pred}}} = -\frac{A_{22} + c_5 p_{k+1}(0) + 2c_6 v_k(T) _{\text{pred}}}{2c_4 p_{k+1}(0) + A_{21} + c_5 v_k(T) _{\text{pred}}}$$

#### 6.5.4 Local Stability Condition

**Local stability** around the equilibrium requires:
$$|\lambda_i(\mathbf{J})| < 1 \quad \text{for all eigenvalues } \lambda_i$$

**Note**: The implicit controller creates strong coupling between position and velocity dynamics, potentially affecting stability margins significantly.

### 6.6 Enhanced Feedback Design

The quadratic controller may require additional feedback for robustness against:
- Quadratic coefficient uncertainties
- Multiple solution branches of the quadratic equation  
- Regions where the discriminant approaches zero

**Modified Controller with Feedback:**
$$v_{\text{des}}' = v_{\text{des}} - K_p(v_k(0) - v_{\text{des}}) - K_d(p_k(0) - p_{\text{eq}})$$

Then solve the quadratic equation using $v_{\text{des}}'$ instead of $v_{\text{des}}$:
$$c_4 p_{k+1}(0)^2 + [A_{21} + c_5 v_k(T) _{\text{pred}}] p_{k+1}(0) + [A_{22} v_k(T) _{\text{pred}} + c_6 v_k(T) _{\text{pred}}^2 - v_{\text{des}}'] = 0$$

**Root Selection Strategy:**
- Choose the root that minimizes step length deviation from nominal
- Implement safeguards when discriminant becomes negative
- Consider switching to linear controller near bifurcation points

### 6.7 Practical Stability Assessment

#### 6.7.1 Numerical Analysis Steps

1. **Find Equilibrium**: Solve the coupled nonlinear equilibrium equations numerically
2. **Compute Jacobian**: Evaluate implicit derivatives at the equilibrium point  
3. **Eigenvalue Analysis**: Check if $|\lambda_i| < 1$ for local stability
4. **Discriminant Analysis**: Ensure quadratic equation has real solutions in operating region
5. **Basin of Attraction**: Use simulation to estimate the region of attraction
6. **Sensitivity Analysis**: Assess robustness to parameter variations and root selection

#### 6.7.2 Implementation Considerations

**Advantages of the Nonlinear Model:**
- Improved prediction accuracy
- Better tracking performance  
- Reduced model-plant mismatch
- More sophisticated control strategies

**Challenges:**
- Multiple equilibrium points may exist
- Quadratic equation may have no real solutions
- Root selection affects stability properties
- Limited basin of attraction
- Increased computational complexity
- Parameter sensitivity
- More complex stability analysis

#### 6.7.3 Computational Concerns

**Real-time Implementation:**
- Pre-compute discriminant validity regions
- Use analytical derivatives for Jacobian computation
- Implement fallback to linear controller when quadratic fails
- Consider lookup tables for root selection in critical regions

### 6.8 Stability Assessment Algorithm

```pseudocode
function assess_nonlinear_quadratic_stability(A, f_coeffs, v_des):
    // Step 1: Find equilibrium point
    [p_eq, u_eq] = solve_coupled_equilibrium(A, f_coeffs, v_des)
    
    // Step 2: Check discriminant validity
    discriminant_valid = check_discriminant_region(A, f_coeffs, p_eq, v_des)
    
    // Step 3: Compute Jacobian with implicit differentiation
    J = compute_implicit_jacobian(A, f_coeffs, p_eq, v_des)
    
    // Step 4: Check local stability  
    eigenvals = eigenvalues(J)
    is_locally_stable = all(abs(eigenvals) < 1)
    
    // Step 5: Estimate basin of attraction
    basin_size = estimate_basin_of_attraction(A, f_coeffs, p_eq, v_des)
    
    return {is_locally_stable, eigenvals, basin_size, discriminant_valid}
end
```

## 7. Summary

The linear inverted pendulum stepping controller provides an elegant framework for bipedal walking control. While the feedforward component achieves perfect velocity tracking within each step, the discrete stepping nature creates step-to-step instability that requires feedback stabilization. The addition of velocity feedback with properly chosen gains ensures stable walking behavior.

The enhanced nonlinear model with quadratic terms offers improved prediction accuracy but introduces additional complexity in stability analysis. The system must be analyzed using nonlinear techniques including equilibrium point analysis, linearization, and numerical methods to assess the basin of attraction.

The key insight is that the kinematic constraint between foot placement and COM position creates a coupling that can be exploited for control, but this same coupling introduces instability that must be actively controlled through feedback. The nonlinear extensions require careful analysis to ensure robust performance across the expected operating range. 
