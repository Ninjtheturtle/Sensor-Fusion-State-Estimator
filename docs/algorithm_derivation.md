# ESKF Algorithm Derivation

## 1. State Representation

### Nominal State (on the manifold)

The nominal state is a 16-element structure (not a flat vector due to the quaternion):

```
x = { p ∈ ℝ³,   position [m]
      v ∈ ℝ³,   velocity [m/s]
      q ∈ S³,   unit quaternion (body-to-world rotation)
      b_g ∈ ℝ³, gyroscope bias [rad/s]
      b_a ∈ ℝ³, accelerometer bias [m/s²] }
```

### Error State (in the tangent space)

The 15-element error state lives in the tangent space of the nominal state:

```
δx = [δp, δv, δθ, δb_g, δb_a]  ∈ ℝ¹⁵
```

`δθ ∈ ℝ³` is a rotation vector (axis × angle). The quaternion error is:
```
δq = [1, δθ/2]  (small angle, normalized)
```

### Why ESKF over Standard EKF?

1. Error states remain **small** → linearization is more accurate
2. Quaternion handled **multiplicatively** (on the manifold) → no singularities
3. Covariance lives in ℝ¹⁵ → well-conditioned matrices
4. After each update, error state is **reset to zero** → relinearization at nominal

---

## 2. IMU Sensor Model

Raw IMU measurements include bias and white noise:

```
a_m = R^T(a_world - g) + b_a + n_a,    n_a ~ N(0, σ²_a·I)
ω_m = ω_body + b_g + n_g,              n_g ~ N(0, σ²_g·I)
```

Bias evolves as a random walk:
```
ḃ_a = n_rw_a,    n_rw_a ~ N(0, σ²_rw_a·I)
ḃ_g = n_rw_g,    n_rw_g ~ N(0, σ²_rw_g·I)
```

Bias-corrected measurements:
```
a_c = a_m - b_a
ω_c = ω_m - b_g
```

---

## 3. Process Model (IMU Prediction)

### Continuous-Time Nominal Kinematics

```
ṗ = v
v̇ = R · a_c + g                          (g = [0,0,-9.81]^T)
q̇ = q ⊗ [0, ω_c/2]^T                    (quaternion kinematic)
ḃ_g = 0  (held constant, noise-driven only)
ḃ_a = 0
```

### RK4 Integration

For accuracy at 200 Hz (dt = 0.005 s), fourth-order Runge-Kutta is used:

```
k₁ = f(xₙ, imu)
k₂ = f(xₙ + dt/2·k₁, imu)
k₃ = f(xₙ + dt/2·k₂, imu)
k₄ = f(xₙ + dt·k₃, imu)
xₙ₊₁ = xₙ + dt/6·(k₁ + 2k₂ + 2k₃ + k₄)
```

The quaternion is treated as a 4-vector during RK4 substeps and renormalized at the end.

### Continuous-Time Error-State Dynamics F (15×15)

Linearizing the kinematics around the nominal state:

```
F = ∂f/∂δx|_{nominal}

     δp   δv      δθ           δb_g   δb_a
δp [ 0    I₃      0             0      0   ]
δv [ 0    0      -R·[a_c]×     -0     -R   ]
δθ [ 0    0      -[ω_c]×       -I₃     0   ]
δb_g[0    0       0             0      0   ]
δb_a[0    0       0             0      0   ]
```

Where `[v]×` is the 3×3 skew-symmetric matrix of vector `v`.

### Discrete State Transition (second-order)

```
Φ = I + F·dt + (F·dt)²/2
```

### Noise Input Matrix G (15×12)

Maps 12 noise inputs to 15 error state dimensions:

```
         n_a     n_g    n_rw_g   n_rw_a
δp  [    0        0       0        0   ]
δv  [    R        0       0        0   ]
δθ  [    0        I       0        0   ]
δb_g[    0        0       I        0   ]
δb_a[    0        0       0        I   ]
```

### Covariance Propagation

Continuous noise covariance:
```
Q_c = diag(σ²_a·I, σ²_g·I, σ²_rw_g·I, σ²_rw_a·I)  (12×12)
```

Discrete propagation:
```
P_{k+1} = Φ·P_k·Φ^T + G·dt·Q_c·(G·dt)^T
```

---

## 4. Measurement Models

### GPS Position Update (3×15 H matrix)

```
z_gps = p + noise,    noise ~ N(0, R_gps)

H_gps = [I₃ | 0 | 0 | 0 | 0]   (selects position block)
```

**Mahalanobis gate** (outlier rejection):
```
S = H·P·H^T + R_gps          (3×3 innovation covariance)
d² = r^T·S⁻¹·r               (Mahalanobis distance squared)
Accept if d² ≤ χ²(3, 0.95) = 7.815
```

**Kalman gain:**
```
K = P·H^T·S⁻¹    (15×3)
```

**Joseph form update** (numerically stable):
```
P_new = (I - K·H)·P·(I - K·H)^T + K·R_gps·K^T
```

### Odometry Velocity Update (3×15 H matrix)

```
z_odom = v + noise,    noise ~ N(0, R_odom)

H_odom = [0 | I₃ | 0 | 0 | 0]   (selects velocity block)
```

Same Kalman update structure as GPS.

---

## 5. Error Injection and Reset

After each measurement update, the accumulated error state is injected into the nominal state:

```
p   ← p + δp
v   ← v + δv
q   ← q ⊗ rotVecToQuat(δθ)    (multiplicative, right-multiply)
b_g ← b_g + δb_g
b_a ← b_a + δb_a
δx  ← 0                        (reset to zero)
```

**Why right-multiply?** The error `δθ` represents a body-frame correction. Left-multiplication would apply a world-frame correction.

**Why multiplicative?** Quaternions lie on the unit sphere S³ (a manifold). Additive correction `q + δq` would leave the manifold. Multiplicative correction `q ⊗ δq` stays on the manifold.

---

## 6. Quaternion Kinematics

### Quaternion convention (Eigen)

Eigen uses the (w, x, y, z) convention. The quaternion q represents the rotation from body to world:

```
v_world = R(q) · v_body
```

### Integration

```
q_dot = 0.5 · q ⊗ Quaternion(0, ω_c)
```

In matrix form using the 4×4 Omega matrix, but implemented via Eigen's product.

### Small-angle approximation

For the error injection step with small `δθ`:
```
δq = [1, δθ.x/2, δθ.y/2, δθ.z/2].normalized()
```

For exact conversion at arbitrary angles:
```
angle = |δθ|
δq = [cos(angle/2), sin(angle/2)/angle · δθ]
```

---

## 7. Numerical Stability Measures

1. **Symmetrization after each step**: `P = (P + P^T) / 2`
2. **Joseph form**: `P = (I-KH)·P·(I-KH)^T + K·R·K^T` instead of `(I-KH)·P`
3. **Second-order Phi**: captures curvature at 200 Hz
4. **Quaternion renormalization** after every RK4 step and inject
5. **safeInverse3x3**: uses Cholesky, falls back to LU

---

## 8. References

- **Primary reference**: Sola, J. (2017). *Quaternion kinematics for the error-state Kalman filter*. arXiv:1711.02508. [Link](https://arxiv.org/abs/1711.02508)
- Forster, C. et al. (2017). *On-Manifold Preintegration for Real-Time Visual-Inertial Odometry*. IEEE T-RO.
- Trawny, N., Roumeliotis, S.I. (2005). *Indirect Kalman Filter for 3D Attitude Estimation*. Tech Report.
