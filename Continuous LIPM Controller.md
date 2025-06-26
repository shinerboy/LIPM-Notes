# LIMP Stepping Controller with Predicted States and Prediction Error Feedback

## Overview

This document presents a hybrid discrete stepping controller for bipedal robots based on the Linear Inverted Pendulum Model (LIMP). This controller uses **predicted end-of-step states** for feedforward control and **prediction error** (current vs predicted) for feedback. This architecture combines the computational efficiency of prediction-based feedforward with the adaptive benefits of prediction error feedback.

## 1. Linear Inverted Pendulum Model

The LIMP represents a humanoid robot as a point mass at the center of mass (COM) moving under the influence of gravity and ground reaction forces. For a robot with COM height $h$ above the ground, the continuous-time dynamics are:

$$\ddot{p}(t) = \frac{g}{h} p(t)$$

Where $p(t)$ is the COM position relative to the stance foot.

### 1.1 Discrete-Time Solution

For a step duration $T$, the discrete-time state transition is:

$$\mathbf{s}(t=T) = \mathbf{A} \mathbf{s}(t=0)$$

Where:

$$\mathbf{s} = \begin{bmatrix} p \\ v \end{bmatrix}, \quad \mathbf{A} = \begin{bmatrix} \cosh(\omega T) & \frac{1}{\omega}\sinh(\omega T) \\ \omega \sinh(\omega T) & \cosh(\omega T) \end{bmatrix}$$

And $\omega = \sqrt{g/h}$ is the natural frequency of the inverted pendulum.

## 2. Hybrid Predicted State Controller Design

### 2.1 Problem Setup

Let:
- $p_k(0)$, $v_k(0)$ = measured COM position and velocity at start of step $k$ (relative to stance foot)
- $p_k(T) _{\text{curr}}$, $v_k(T) _{\text{curr}}$ = **current** COM position and velocity at end of step $k$
- $p_k(T) _{\text{pred}}$, $v_k(T) _{\text{pred}}$ = **predicted** COM position and velocity at end of step $k$
- $u$ = step length (distance between consecutive feet)
- $v_{\text{des}}$ = desired walking velocity

### 2.2 Stepping Constraint

The relationship between consecutive foot placements and COM position is:

$$p_k(0) = u - p_{k-1}(T) _{\text{curr}}$$

This constraint couples the foot placement decision with the actual current COM dynamics.

### 2.3 Prediction Model

At the start of step $k$, we predict the final state using the LIMP model:

$$\begin{bmatrix} p_k(T) _{\text{pred}} \\ v_k(T) _{\text{pred}} \end{bmatrix} = \mathbf{A} \begin{bmatrix} p_k(0) \\ v_k(0) \end{bmatrix}$$

### 2.4 Predicted State Feedforward Controller

The feedforward controller uses the **predicted** end-of-step velocity to determine step length:

$$u_{\text{ff}} = p_k(T)_ {\text{pred}} + \frac{v_{\text{des}} - A_{22} \cdot v_k(T)_ {\text{pred}}}{A_{21}}$$

**Key Insight**: This approach uses model predictions for planning while maintaining feedback adaptation capabilities.

### 2.5 Prediction Error Feedback Controller

The feedback controller corrects for prediction errors:

$$u = u_{\text{ff}} - K_p(v_k(T) _{\text{curr}} - v_k(T) _{\text{pred}}) - K_d(p_k(T) _{\text{curr}} - p_k(T) _{\text{pred}})$$

**Complete Controller**:

$$u = p_k(T)_ {\text{pred}} + \frac{v_{\text{des}} - A_{22} \cdot v_k(T)_ {\text{pred}}}{A_{21}} - K_p(v_k(T) _{\text{curr}} - v_k(T) _{\text{pred}}) - K_d(p_k(T) _{\text{curr}} - p_k(T) _{\text{pred}})$$

### 2.6 Controller Philosophy

**Feedforward Component**:
- Uses predicted states → computationally efficient, can plan ahead
- Based on LIMP model → leverages theoretical understanding

**Feedback Component**:
- Corrects for prediction errors → improves model accuracy over time
- $K_p$ corrects velocity prediction errors
- $K_d$ corrects position prediction errors

## 3. Stability Analysis

### 3.1 Step-to-Step Dynamics with Hybrid States

The actual COM state at the end of step $k$ is:

$$\begin{bmatrix} p_k(T) _{\text{curr}} \\ v_k(T) _{\text{curr}} \end{bmatrix} = \mathbf{A} \begin{bmatrix} p_k(0) \\ v_k(0) \end{bmatrix} + \mathbf{d}_k$$

Where $\mathbf{d}_k$ represents model uncertainties, disturbances, and unmodeled dynamics.

The initial conditions for step $k+1$ are:

$$p_{k+1}(0) = u - p_k(T)_ {\text{curr}}$$

$$v_{k+1}(0) = v_k(T) _{\text{curr}}$$

### 3.2 Feedforward Stability Analysis

For the feedforward controller alone, we need to analyze the step-to-step velocity dynamics. The predicted end-of-step velocity for step $k+1$ is:

$$v_{k+1}(T)_ {\text{pred}} = A_{21}p_{k+1}(0) + A_{22}v_{k+1}(0)$$

Substituting the feedforward controller:

$$p_{k+1}(0) = u - p_k(T)_ {\text{curr}} = p_k(T)_ {\text{pred}} + \frac{v_{\text{des}} - A_{22} \cdot v_k(T)_ {\text{pred}}}{A_{21}} - p_k(T) _{\text{curr}}$$

This gives:

$$v_{k+1}(T)_ {\text{pred}} = A_{21}\[p_k(T)_ {\text{pred}} - p_k(T)_ {\text{curr}} + \frac{v_{\text{des}} - A_{22} \cdot v_k(T)_ {\text{pred}}}{A_{21}}\] + A_{22}v_k(T) _{\text{curr}}$$

Simplifying:

$$v_{k+1}(T)_ {\text{pred}} = A_{21}(p_k(T)_ {\text{pred}} - p_k(T)_ {\text{curr}}) + (v_{\text{des}} - A_{22} \cdot v_k(T)_ {\text{pred}}) + A_{22}v_k(T) _{\text{curr}}$$

$$v_{k+1}(T)_ {\text{pred}} = v_{\text{des}} + A_{21}(p_k(T)_ {\text{pred}} - p_k(T)_ {\text{curr}}) + A_{22}(v_k(T) _{\text{curr}} - v_k(T) _{\text{pred}})$$

**Result**: The feedforward controller achieves $v_{k+1}(T)_ {\text{pred}} = v_{\text{des}}$ only if there are no prediction errors. Otherwise, prediction errors propagate and can lead to instability.

### 3.3 Feedforward Instability

Define prediction errors:

$$e_{p,k} = p_k(T) _{\text{curr}} - p_k(T) _{\text{pred}}$$

$$e_{v,k} = v_k(T) _{\text{curr}} - v_k(T) _{\text{pred}}$$

The velocity prediction for step $k+1$ becomes:

$$v_{k+1}(T)_ {\text{pred}} = v_{\text{des}} - A_{21}e_{p,k} + A_{22}e_{v,k}$$

If prediction errors persist, the system will deviate from the desired velocity, potentially leading to instability. This is why feedback is essential.

### 3.4 Combined System with Feedback

The complete controller modifies the step length:

$$u = u_{\text{ff}} - K_p e_{v,k} - K_d e_{p,k}$$

This creates the modified initial condition:

$$p_{k+1}(0) = u_{\text{ff}} - K_p e_{v,k} - K_d e_{p,k} - p_k(T) _{\text{curr}}$$

Substituting $u_{\text{ff}}$:

$$p_{k+1}(0) = p_k(T)_ {\text{pred}} + \frac{v_{\text{des}} - A_{22} \cdot v_k(T)_ {\text{pred}}}{A_{21}} - K_p e_{v,k} - K_d e_{p,k} - p_k(T) _{\text{curr}}$$

$$p_{k+1}(0) = -e_{p,k} + \frac{v_{\text{des}} - A_{22} \cdot v_k(T)_ {\text{pred}}}{A_{21}} - K_p e_{v,k} - K_d e_{p,k}$$

$$p_{k+1}(0) = \frac{v_{\text{des}} - A_{22} \cdot v_k(T)_ {\text{pred}}}{A_{21}} - (1 + K_d) e_{p,k} - K_p e_{v,k}$$

### 3.5 Prediction Error Dynamics

The prediction error for step $k+1$ depends on the model accuracy and the actual disturbances. If we assume the model is perfect except for external disturbances:

$$e_{p,k+1} = \text{position error due to disturbances} + \text{propagation of control errors}$$

$$e_{v,k+1} = \text{velocity error due to disturbances} + \text{propagation of control errors}$$

The feedback terms help minimize the propagation of control errors, making the system more robust to prediction errors.

## 4. Feedback Gain Design

### 4.1 Prediction Error Feedback Philosophy

This controller uses **prediction error feedback** rather than trajectory tracking:

- **Traditional**: Feedback corrects deviation from desired trajectory
- **This Controller**: Feedback corrects model prediction errors while using predictions for feedforward

### 4.2 Error Dynamics Analysis for Gain Design

To find optimal feedback gains, we need to analyze how prediction errors evolve step-to-step and design gains to stabilize the error dynamics.

#### 4.2.1 Prediction Error Evolution

Define prediction errors at step $k$:

$$e_{p,k} = p_k(T)_ {\text{curr}} - p_k(T)_ {\text{pred}}$$

$$e_{v,k} = v_k(T)_ {\text{curr}} - v_k(T)_ {\text{pred}}$$

The actual end-of-step state is:

$$\begin{bmatrix} p_k(T) _{\text{curr}} \\ v_k(T) _{\text{curr}} \end{bmatrix} = \mathbf{A} \begin{bmatrix} p_k(0) \\ v_k(0) \end{bmatrix} + \mathbf{d}_k$$

The predicted end-of-step state is:

$$\begin{bmatrix} p_k(T) _{\text{pred}} \\ v_k(T) _{\text{pred}} \end{bmatrix} = \mathbf{A} \begin{bmatrix} p_k(0) \\ v_k(0) \end{bmatrix}$$

Therefore, the prediction error is:

$$\begin{bmatrix} e_{p,k} \\ e_{v,k} \end{bmatrix} = \mathbf{d}_k$$

#### 4.2.2 Step-to-Step Error Dynamics

For step $k+1$, the initial conditions are:

$$p_{k+1}(0) = u - p_k(T)_ {\text{curr}}$$

$$v_{k+1}(0) = v_k(T) _{\text{curr}}$$

Substituting the complete controller:

$$p_{k+1}(0) = p_k(T)_ {\text{pred}} + \frac{v_{\text{des}} - A_{22} v_k(T)_ {\text{pred}}}{A_{21}} - K_p e_{v,k} - K_d e_{p,k} - p_k(T) _{\text{curr}}$$

$$p_{k+1}(0) = -e_{p,k} + \frac{v_{\text{des}} - A_{22} v_k(T)_ {\text{pred}}}{A_{21}} - K_p e_{v,k} - K_d e_{p,k}$$

$$p_{k+1}(0) = \frac{v_{\text{des}} - A_{22} v_k(T)_ {\text{pred}}}{A_{21}} - (1 + K_d) e_{p,k} - K_p e_{v,k}$$

The predicted next state is:

$$\begin{bmatrix} p_{k+1}(T)_ {\text{pred}} \\ v_{k+1}(T)_ {\text{pred}} \end{bmatrix} = \mathbf{A} \begin{bmatrix} p_{k+1}(0) \\ v_{k+1}(0) \end{bmatrix}$$

$$= \mathbf{A} \begin{bmatrix} \frac{v_{\text{des}} - A_{22} v_k(T)_ {\text{pred}}}{A_{21}} - (1 + K_d) e_{p,k} - K_p e_{v,k} \\ v_k(T) _{\text{curr}} \end{bmatrix}$$

#### 4.2.3 Error Propagation Analysis

The actual next state will be:

$$\begin{bmatrix} p_{k+1}(T)_ {\text{curr}} \\ v_{k+1}(T)_ {\text{curr}} \end{bmatrix} = \mathbf{A} \begin{bmatrix} p_{k+1}(0) \\ v_{k+1}(0) \end{bmatrix} + \mathbf{d}_{k+1}$$

The predicted next state is:

$$\begin{bmatrix} p_{k+1}(T)_ {\text{pred}} \\ v_{k+1}(T)_ {\text{pred}} \end{bmatrix} = \mathbf{A} \begin{bmatrix} p_{k+1}(0) \\ v_{k+1}(0) \end{bmatrix}$$

**Step-by-Step Derivation of Error Propagation:**

The prediction error at step $k+1$ is:

$$\begin{bmatrix} e_{p,k+1} \\ e_{v,k+1} \end{bmatrix} = \begin{bmatrix} p_{k+1}(T)_ {\text{curr}} \\ v_{k+1}(T)_ {\text{curr}} \end{bmatrix} - \begin{bmatrix} p_{k+1}(T)_ {\text{pred}} \\ v_{k+1}(T) _{\text{pred}} \end{bmatrix}$$

Substituting the state equations:

$$\begin{bmatrix} e_{p,k+1} \\ e_{v,k+1} \end{bmatrix} = \left(\mathbf{A} \begin{bmatrix} p_{k+1}(0) \\ v_{k+1}(0) \end{bmatrix} + \mathbf{d}_ {k+1}\right) - \mathbf{A} \begin{bmatrix} p_{k+1}(0) \\ v_{k+1}(0) \end{bmatrix}$$

$$= \mathbf{d}_{k+1}$$

**But wait!** This assumes the initial conditions $p_{k+1}(0)$ and $v_{k+1}(0)$ are the same for both actual and predicted trajectories. However, our controller modifies $p_{k+1}(0)$ based on feedback, so we need to be more careful.

**Detailed Analysis:**

The **actual** initial conditions for step $k+1$ are:

$$p_{k+1}(0) = u - p_k(T)_ {\text{curr}}$$

$$v_{k+1}(0) = v_k(T) _{\text{curr}}$$

Where $u$ includes feedback:

$$u = p_k(T)_ {\text{pred}} + \frac{v_{\text{des}} - A_{22} v_k(T)_ {\text{pred}}}{A_{21}} - K_p e_{v,k} - K_d e_{p,k}$$

The **predicted** initial conditions for step $k+1$ would be (if we used the predicted states):

$$p_{k+1}(0)_ {\text{pred}} = u_{\text{ff}} - p_k(T)_ {\text{pred}}$$

$$v_{k+1}(0)_{\text{pred}} = v_k(T) _{\text{pred}}$$

Where $u_{\text{ff}}$ is the feedforward-only controller:

$$u_{\text{ff}} = p_k(T)_ {\text{pred}} + \frac{v_{\text{des}} - A_{22} v_k(T)_ {\text{pred}}}{A_{21}}$$

**Computing the difference in initial conditions:**

$$p_{k+1}(0) - p_{k+1}(0)_ {\text{pred}} = (u - p_k(T)_ {\text{curr}}) - (u_{\text{ff}} - p_k(T) _{\text{pred}})$$

$$= (u - u_{\text{ff}}) + (p_k(T) _{\text{pred}} - p_k(T) _{\text{curr}})$$

$$= (-K_p e_{v,k} - K_d e_{p,k}) + (-e_{p,k})$$

$$= -(1 + K_d) e_{p,k} - K_p e_{v,k}$$

$$v_{k+1}(0) - v_{k+1}(0)_ {\text{pred}} = v_ k(T)_ {\text{curr}} - v_k(T)_ {\text{pred}} = e_{v,k}$$

**Therefore:**

$$\begin{bmatrix} p_{k+1}(0) \\ v_{k+1}(0) \end{bmatrix} = \begin{bmatrix} p_{k+1}(0)_ {\text{pred}} \\ v_{k+1}(0)_ {\text{pred}} \end{bmatrix} + \begin{bmatrix} -(1 + K_d) e_{p,k} - K_p e_{v,k} \\ e_{v,k} \end{bmatrix}$$

**Now we can compute the prediction error:**

$$\begin{bmatrix} e_{p,k+1} \\ e_{v,k+1} \end{bmatrix} = \begin{bmatrix} p_{k+1}(T)_ {\text{curr}} \\ v_{k+1}(T)_ {\text{curr}} \end{bmatrix} - \begin{bmatrix} p_{k+1}(T)_ {\text{pred}} \\ v_{k+1}(T) _{\text{pred}} \end{bmatrix}$$

The actual trajectory:

$$\begin{bmatrix} p_{k+1}(T)_ {\text{curr}} \\ v_{k+1}(T)_ {\text{curr}} \end{bmatrix} = \mathbf{A} \begin{bmatrix} p_{k+1}(0) \\ v_{k+1}(0) \end{bmatrix} + \mathbf{d}_{k+1}$$

The predicted trajectory:

$$\begin{bmatrix} p_{k+1}(T)_ {\text{pred}} \\ v_{k+1}(T)_ {\text{pred}} \end{bmatrix} = \mathbf{A} \begin{bmatrix} p_{k+1}(0)_ {\text{pred}} \\ v_{k+1}(0)_{\text{pred}} \end{bmatrix}$$

**Substituting:**

$$\begin{bmatrix} e_{p,k+1} \\ e_{v,k+1} \end{bmatrix} = \mathbf{A} \begin{bmatrix} p_{k+1}(0) \\ v_{k+1}(0) \end{bmatrix} + \mathbf{d}_ {k+1} - \mathbf{A} \begin{bmatrix} p_{k+1}(0)_ {\text{pred}} \\ v_{k+1}(0)_{\text{pred}} \end{bmatrix}$$

$$= \mathbf{A} \left(\begin{bmatrix} p_{k+1}(0) \\ v_{k+1}(0) \end{bmatrix} - \begin{bmatrix} p_{k+1}(0)_ {\text{pred}} \\ v_{k+1}(0)_ {\text{pred}} \end{bmatrix}\right) + \mathbf{d}_{k+1}$$

$$= \mathbf{A} \begin{bmatrix} -(1 + K_d) e_{p,k} - K_p e_{v,k} \\ e_{v,k} \end{bmatrix} + \mathbf{d}_{k+1}$$

**Rearranging:**

$$\begin{bmatrix} e_{p,k+1} \\ e_{v,k+1} \end{bmatrix} = \mathbf{A} \begin{bmatrix} 0 \\ e_{v,k} \end{bmatrix} - \mathbf{A} \begin{bmatrix} (1 + K_d) e_{p,k} + K_p e_{v,k} \\ 0 \end{bmatrix} + \mathbf{d}_{k+1}$$

Expanding this:

$$\begin{bmatrix} e_{p,k+1} \\ e_{v,k+1} \end{bmatrix} = \begin{bmatrix} A_{12} e_{v,k} - A_{11}((1 + K_d) e_{p,k} + K_p e_{v,k}) \\ A_{22} e_{v,k} - A_{21}((1 + K_d) e_{p,k} + K_p e_{v,k}) \end{bmatrix} + \mathbf{d}_{k+1}$$

$$= \begin{bmatrix} -A_{11}(1 + K_d) & A_{12} - A_{11}K_p \\ -A_{21}(1 + K_d) & A_{22} - A_{21}K_p \end{bmatrix} \begin{bmatrix} e_{p,k} \\ e_{v,k} \end{bmatrix} + \mathbf{d}_{k+1}$$

#### 4.2.4 Error Dynamics Matrix

Define the error evolution matrix:

$$\mathbf{E} = \begin{bmatrix} -A_{11}(1 + K_d) & A_{12} - A_{11}K_p \\ -A_{21}(1 + K_d) & A_{22} - A_{21}K_p \end{bmatrix}$$

The error dynamics are:

$$\begin{bmatrix} e_{p,k+1} \\ e_{v,k+1} \end{bmatrix} = \mathbf{E} \begin{bmatrix} e_{p,k} \\ e_{v,k} \end{bmatrix} + \mathbf{d}_{k+1}$$

#### 4.2.5 Stability Condition

For the error dynamics to be stable, all eigenvalues of $\mathbf{E}$ must have magnitude less than 1:

$$|\lambda_i(\mathbf{E})| < 1 \quad \forall i$$

#### 4.2.6 Deadbeat Gain Design

For **deadbeat control** (fastest possible convergence), we want:

$$\mathbf{E} = \mathbf{0}$$

This requires:

$$-A_{11}(1 + K_d) = 0 \Rightarrow K_d = -1 + \frac{0}{A_{11}} = -1$$

$$A_{12} - A_{11}K_p = 0 \Rightarrow K_p = \frac{A_{12}}{A_{11}}$$

$$-A_{21}(1 + K_d) = 0 \Rightarrow K_d = -1$$ (consistent)

$$A_{22} - A_{21}K_p = 0 \Rightarrow K_p = \frac{A_{22}}{A_{21}}$$


For consistency, we need:

$$\frac{A_{12}}{A_{11}} = \frac{A_{22}}{A_{21}}$$

This gives us the **deadbeat gains**:

$$K_d = -1$$

$$K_p = \frac{A_{22}}{A_{21}} = \frac{\cosh(\omega T)}{\omega \sinh(\omega T)}$$

**Note**: $K_d = -1$ means we're adding the position error (rather than subtracting), which makes sense because we want to compensate for underprediction.

#### 4.2.7 Practical Deadbeat Gains

For typical values ($\omega T = 0.313$):

$$K_p = \frac{\cosh(0.313)}{\omega \sinh(0.313)} = \frac{1.049}{3.13 \times 0.317} \approx 1.06$$

$$K_d = -1$$

#### 4.2.8 Conservative Gain Design

For more conservative behavior, we can use:

$$K_p = \alpha \frac{A_{22}}{A_{21}}, \quad K_d = \alpha(-1)$$

Where $0 < \alpha < 1$ provides a slower but more robust response.

#### 4.2.9 Pole Placement Method

Alternatively, we can choose desired eigenvalues $\lambda_1, \lambda_2$ for the error dynamics and solve:

$$\det(\mathbf{E} - \lambda \mathbf{I}) = (\lambda - \lambda_1)(\lambda - \lambda_2)$$

This gives a system of equations to solve for $K_p$ and $K_d$.

#### 4.2.10 Recommended Gain Selection Strategy

1. **Start with deadbeat gains** as the theoretical optimum
2. **Scale down by 0.5-0.8** for robustness: $K_p = 0.7 \times 1.06 \approx 0.74$, $K_d = 0.7 \times (-1) = -0.7$
3. **Monitor prediction errors** and adjust based on performance
4. **Increase gains gradually** if prediction errors persist
5. **Decrease gains** if the system becomes oscillatory

### 4.3 Stability Considerations

The stability of the hybrid controller depends on:

1. **Feedforward Accuracy**: More accurate predictions → smaller required feedback gains
2. **Feedback Responsiveness**: Faster correction of prediction errors
3. **Disturbance Magnitude**: Larger disturbances require more aggressive feedback

The system is generally stable if:
- The LIMP model is reasonably accurate
- Feedback gains are properly tuned
- Prediction errors don't grow faster than feedback can correct them

## 5. Implementation Considerations

### 5.1 Measurement Requirements

This controller requires both **prediction capability** and **measurement accuracy**:
- **Prediction**: Accurate LIMP model parameters ($\omega$, step time $T$)
- **Current State**: COM position and velocity at end of each step
- **Timing**: Precise step timing for accurate predictions

### 5.2 Typical Parameter Values

For a humanoid robot with:
- Center of mass height: $h = 1.0$ m
- Time step: $T = 0.1$ s
- Natural frequency: $\omega = \sqrt{9.81/1.0} \approx 3.13$ rad/s

The state transition matrix becomes:

$$\mathbf{A} \approx \begin{bmatrix} 1.049 & 0.317 \\\ 0.993 & 1.049 \end{bmatrix}$$

**Deadbeat feedback gains** (fastest convergence):

$$K_p = \frac{A_{22}}{A_{21}} = \frac{1.049}{0.993} \approx 1.06$$

$$K_d = -1$$

**Conservative feedback gains** (robust performance):

$$K_p \approx 0.7 \times 1.06 = 0.74, \quad K_d \approx 0.7 \times (-1) = -0.7$$

### 5.3 Advantages

1. **Computational Efficiency**: Feedforward uses simple prediction, not measurement waiting
2. **Predictive Planning**: Can plan ahead using model predictions
3. **Error Correction**: Adapts to model inaccuracies through feedback
4. **Fast Response**: Doesn't wait for measurements to compute feedforward
5. **Model Utilization**: Leverages theoretical understanding while remaining adaptive

### 5.4 Challenges

1. **Model Dependence**: Feedforward quality depends on LIMP model accuracy
2. **Parameter Sensitivity**: Requires good estimates of $\omega$ and $T$
3. **Prediction Error Accumulation**: Poor predictions can lead to instability
4. **Gain Tuning**: Requires careful balance between feedforward and feedback
5. **Measurement Quality**: Still needs good current state measurements for feedback

### 5.5 Practical Recommendations

1. **Calibrate model parameters** regularly ($h$, $T$, ground properties)
2. **Monitor prediction errors** to assess model quality
3. **Use conservative feedback gains** initially and increase gradually
4. **Implement prediction error logging** for system diagnostics
5. **Consider model adaptation** based on persistent prediction errors

## 6. Comparison with Other Controllers

### 6.1 Traditional Prediction-Based Controller
- **Feedforward**: Uses predicted states ✓ (same)
- **Feedback**: Uses trajectory tracking errors
- **Philosophy**: Plan-then-execute with trajectory correction

### 6.2 Current State Controller
- **Feedforward**: Uses current states
- **Feedback**: Uses prediction errors ✓ (same)
- **Philosophy**: Measure-then-adapt

### 6.3 This Hybrid Controller
- **Feedforward**: Uses predicted states ✓
- **Feedback**: Uses prediction errors ✓
- **Philosophy**: Predict-then-adapt

### 6.4 Performance Characteristics

| Aspect | Traditional | Current State | Hybrid Predicted |
|--------|-------------|---------------|------------------|
| Model Sensitivity | High | Low | Medium |
| Adaptation | Manual | Automatic | Semi-Automatic |
| Computational Speed | Fast | Medium | Fast |
| Prediction Required | Yes | No | Yes |
| Measurement Delay | Low | High | Low |
| Learning Capability | Limited | Built-in | Moderate |

## 7. Extension to Quadratic Nonlinear Models

### 7.1 Nonlinear LIMP Model

For more accurate modeling, we can include quadratic nonlinearities in the LIMP dynamics:

$$\mathbf{s}(T) = \mathbf{A} \mathbf{s}(0) + \mathbf{f}(\mathbf{s}(0))$$

Where the quadratic function is:

$$\mathbf{f}(\mathbf{s}) = \begin{bmatrix} f_1(p,v) \\\ f_2(p,v) \end{bmatrix} = \begin{bmatrix} c_1p^2 + c_2pv + c_3v^2 \\\ c_4p^2 + c_5pv + c_6v^2 \end{bmatrix}$$

This can account for:
- **Centripetal effects**: $c_3v^2$ terms from curved walking
- **Coupling effects**: $c_2pv$ and $c_5pv$ terms from position-velocity interactions
- **Higher-order dynamics**: $c_1p^2$ and $c_4p^2$ terms from nonlinear restoring forces

### 7.2 Nonlinear Feedforward Controller

The feedforward controller becomes more complex as we need to solve a quadratic equation.

**Step 1: Set up the constraint equation**

$$v_{k+1}(T) = v_{\text{des}}$$

**Step 2: Express in terms of initial conditions**

$$A_{21}p_{k+1}(0) + A_{22}v_{k+1}(0) + f_2(p_{k+1}(0), v_{k+1}(0)) = v_{\text{des}}$$

**Step 3: Substitute stepping constraint**

$$p_{k+1}(0) = u - p_k(T)_ {\text{pred}}$$

$$v_{k+1}(0) = v_k(T) _{\text{pred}}$$

**Step 4: Nonlinear equation in $u$**

Let $p_0 = u - p_k(T) _{\text{pred}}$ and $v_0 = v_k(T) _{\text{pred}}$:

$$A_{21}p_0 + A_{22}v_0 + c_4p_0^2 + c_5p_0v_0 + c_6v_0^2 = v_{\text{des}}$$

**Step 5: Quadratic equation in $u$**

Substituting $p_0 = u - p_k(T) _{\text{pred}}$:

$$c_4u^2 + (A_{21} + c_5v_0 - 2c_4p_k(T)_ {\text{pred}})u + \text{(remaining terms)} = v_{\text{des}}$$

**Quadratic coefficients:**

$$a = c_4$$

$$b = A_{21} + c_5v_k(T) _{\text{pred}} - 2c_4p_k(T) _{\text{pred}}$$

$$c = \text{remaining constant terms} - v_{\text{des}}$$

**Solution:**

$$u_{\text{ff}} = \frac{-b \pm \sqrt{b^2 - 4ac}}{2a}$$

Choose the solution that gives reasonable step length (typically the smaller positive root).

### 7.3 Linearized Error Dynamics for Gain Design

For gain design, we linearize the nonlinear system around the operating point.

#### 7.3.1 Jacobian of Nonlinear Function

$$\mathbf{J}_f = \nabla \mathbf{f} = \begin{bmatrix} \frac{\partial f_1}{\partial p} & \frac{\partial f_1}{\partial v} \\\ \frac{\partial f_2}{\partial p} & \frac{\partial f_2}{\partial v} \end{bmatrix} = \begin{bmatrix} 2c_1p + c_2v & c_2p + 2c_3v \\\ 2c_4p + c_5v & c_5p + 2c_6v \end{bmatrix}$$

#### 7.3.2 Linearized State Transition

Around an operating point $\mathbf{s}_{\text{op}} = \[p _{\text{op}}, v _{\text{op}}\]^T$:

$$\mathbf{s}(T) \approx \mathbf{A} \mathbf{s}(0) + \mathbf{f}(\mathbf{s}_ {\text{op}}) + \mathbf{J}_ f(\mathbf{s}_ {\text{op}}) (\mathbf{s}(0) - \mathbf{s}_{\text{op}})$$

$$= (\mathbf{A} + \mathbf{J}_ f(\mathbf{s}_ {\text{op}})) \mathbf{s}(0) + (\mathbf{f}(\mathbf{s}_ {\text{op}}) - \mathbf{J}_ f(\mathbf{s}_ {\text{op}}) \mathbf{s}_{\text{op}})$$

Define the **linearized state transition matrix**:

$$\mathbf{A}_ {\text{lin}} = \mathbf{A} + \mathbf{J}_ f(\mathbf{s}_{\text{op}})$$

#### 7.3.3 Error Dynamics with Linearized Model

The prediction error evolution becomes:

$$\begin{bmatrix} e_{p,k+1} \\\ e_{v,k+1} \end{bmatrix} = \mathbf{E}_ {\text{lin}} \begin{bmatrix} e_{p,k} \\\ e_{v,k} \end{bmatrix} + \mathbf{d}_ {k+1} + \mathbf{\epsilon}_{\text{nonlin}}$$

Where:

$$\mathbf{E}_ {\text{lin}} = \begin{bmatrix} -A_{11,\text{lin}}(1 + K_d) & A_{12,\text{lin}} - A_{11,\text{lin}}K_p \\\ -A_{21,\text{lin}}(1 + K_d) & A_{22,\text{lin}} - A_{21,\text{lin}}K_p \end{bmatrix}$$

And $\mathbf{\epsilon}_{\text{nonlin}}$ represents higher-order nonlinear terms.

#### 7.3.4 Adaptive Operating Point Selection

**Strategy 1: Fixed Operating Point**

Use the desired steady-state: $\mathbf{s}_ {\text{op}} = \[p_{\text{ss}}, v_{\text{des}}\]^T$

**Strategy 2: Moving Operating Point**
Update based on recent states: 
$\mathbf{s}_ {\text{op}} = \alpha \mathbf{s}_ {\text{recent}} + (1-\alpha) \mathbf{s}_{\text{op,prev}}$

**Strategy 3: Prediction-Based Operating Point**
Use the current prediction: 

$\mathbf{s}_{\text{op}} = \[p_k(T) _{\text{pred}}, v_k(T) _{\text{pred}}\]^T$

### 7.4 Gain Design for Nonlinear System

#### 7.4.1 Linearized Deadbeat Gains

Using the linearized system:

$$K_{d,\text{lin}} = -1$$

$$K_{p,\text{lin}} = \frac{A_{22,\text{lin}}}{A_{21,\text{lin}}} = \frac{A_{22} + 2c_6v_{\text{op}} + c_5p_{\text{op}}}{A_{21} + 2c_4p_{\text{op}} + c_5v_{\text{op}}}$$

#### 7.4.2 Robust Gain Design

For nonlinear systems, use more conservative gains to account for modeling uncertainties:

$$K_p = \beta \cdot K_{p,\text{lin}}$$

$$K_d = \beta \cdot K_{d,\text{lin}}$$

Where $\beta = 0.3 \text{ to } 0.7$ provides robustness margin.

#### 7.4.3 Adaptive Gain Scheduling

**Method 1: Operating Point Dependent**

$$K_p(\mathbf{s}) = \frac{A_{22} + 2c_6v + c_5p}{A_{21} + 2c_4p + c_5v}$$

$$K_d(\mathbf{s}) = -1$$

**Method 2: Prediction Error Magnitude Dependent**

$$K_p = K_{p,\text{base}} \cdot \left(1 + \gamma \sqrt{e_{p,k}^2 + e_{v,k}^2}\right)$$

$$K_d = K_{d,\text{base}} \cdot \left(1 + \gamma \sqrt{e_{p,k}^2 + e_{v,k}^2}\right)$$

Where $\gamma$ controls the adaptation rate.

### 7.5 Implementation Considerations for Nonlinear Model

#### 7.5.1 Computational Complexity

**Feedforward Controller:**
- Requires solving quadratic equation at each step
- Need robust root selection algorithm
- Consider numerical conditioning

**Feedback Gains:**
- Jacobian computation: $O(1)$ for quadratic functions
- Matrix operations: $O(1)$ for 2×2 system
- Overall computational cost is still reasonable

#### 7.5.2 Parameter Identification

**Model Identification:**
The coefficients $c_1, ..., c_6$ can be identified using:
- **Least squares fitting** on observed trajectory data
- **Recursive least squares** for online adaptation
- **Extended Kalman filter** for simultaneous state and parameter estimation

**Data Collection:**
Collect data pairs $\{(\mathbf{s}(0), \mathbf{s}(T))\}$ and fit:
$$\mathbf{s}(T) - \mathbf{A}\mathbf{s}(0) = \mathbf{f}(\mathbf{s}(0))$$

#### 7.5.3 Stability Guarantees

**Local Stability:**
- Guaranteed within linearization region
- Monitor prediction errors to ensure validity

**Global Stability:**
- More complex analysis required
- Consider Lyapunov function approach
- Use conservative gains for robustness

#### 7.5.4 Practical Recommendations

1. **Start with linear model** and verify basic functionality
2. **Add quadratic terms gradually** starting with most significant coefficients
3. **Use parameter identification** to determine coefficients from data
4. **Monitor linearization validity** by tracking prediction errors
5. **Implement gain scheduling** based on operating conditions
6. **Use robust numerical methods** for quadratic equation solving

### 7.6 Example: Nonlinear Gain Calculation

For a system with:

- $h = 1.0$ m, $T = 0.1$ s, $\omega = 3.13$ rad/s
- 
- Operating point: $p_{\text{op}} = 0.1$ m, $v_{\text{op}} = 1.0$ m/s
- 
- Nonlinear coefficients: $c_4 = 0.05$, $c_5 = 0.02$, $c_6 = 0.01$

**Linearized transition matrix:**

$$A_{21,\text{lin}} = 0.993 + 2(0.05)(0.1) + (0.02)(1.0) = 0.993 + 0.01 + 0.02 = 1.023$$

$$A_{22,\text{lin}} = 1.049 + (0.02)(0.1) + 2(0.01)(1.0) = 1.049 + 0.002 + 0.02 = 1.071$$

**Deadbeat gains:**

$$K_{p,\text{lin}} = \frac{1.071}{1.023} = 1.047$$

$$K_{d,\text{lin}} = -1$$

**Conservative gains (β = 0.6):**

$$K_p = 0.6 \times 1.047 = 0.628$$

$$K_d = 0.6 \times (-1) = -0.6$$

## 8. Summary

The hybrid predicted state LIMP stepping controller combines the benefits of prediction-based feedforward with adaptive prediction error feedback. This approach provides:

1. **Computational Efficiency**: Uses predictions for fast feedforward computation
2. **Predictive Capability**: Can plan ahead without waiting for measurements
3. **Adaptive Feedback**: Corrects for model inaccuracies and disturbances
4. **Balanced Approach**: Leverages both model knowledge and measurement feedback

The key insight is that using **predicted states for feedforward** maintains computational efficiency and planning capability, while **prediction error feedback** provides the adaptation necessary for robust performance in uncertain environments.

**Controller Equation Summary**:
$$u = p_k(T)_ {\text{pred}} + \frac{v_{\text{des}} - A_{22} \cdot v_k(T)_ {\text{pred}}}{A_{21}} - K_p(v_k(T) _{\text{curr}} - v_k(T) _{\text{pred}}) - K_d(p_k(T) _{\text{curr}} - p_k(T) _{\text{pred}})$$

This controller is particularly suitable for applications where:
- Fast computation is required
- Model predictions are reasonably accurate
- Some adaptation to uncertainties is needed
- Real-time planning capabilities are desired

The hybrid approach represents a middle ground between purely prediction-based and purely measurement-based control strategies, offering computational efficiency with adaptive robustness. 
