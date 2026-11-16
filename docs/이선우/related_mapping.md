# 조사

## 센서

위치 추정은 IMU + GPS가 일반적

GPS → 일단 실내에서 포착이 안될 확률이 높다.

하지만 Encoder + Wifi 로 대처가 가능할수도 있을것 같음

(GPS → wifi fingerprinting or bluetooth AoA, UWB기반 거리 측정)

### wifi fingerprinting

먼저 사전에 reference 할 특정 지점을 수집하고, 실재로 적용할때는 디바이스가 측정한 신호를 바탕으로 사전에 수집한 데이터와 비교하여 현재 위치를 추정한다. → radio map 을 수집해야하는 거보면 신호 값의 강약을 기반으로  추정하는 듯.

각 reference point에 esp32 와같은 transmitter가 필요할것 같다 → 가격적으로 불가능할듯

---

AoA, UWB, Wifi finger printing 다 비슷한 단점을 가지고 있다.

사전에 파이프를 매핑해 놓고 매핑 기반의 방식, multi encoder, 카메라 기반의 방식이 있다 ← 다 gps 대용

## 방식(GPS + IMU 기반)

위치 추정에서 sensor fusion 방식은 kalman filter, extended kalman filter, unscented kalman filter, particle filter 등이 있다.

큰 방식은 predict → update

### kalman filter (linear model)

**Full form (multiple variables)**

Real robots track a state vector, for example position, velocity and heading, so the scalars become matrices:

```
Predict:  x = F x + B u          P = F P Fᵀ + Q
Update:   K = P Hᵀ (H P Hᵀ + R)⁻¹
          x = x + K (z − H x)    P = (I − K H) P
```

- `F`: how the state evolves (motion model)
- `B u`: control input (encoder, IMU)
- `H`: how the state maps to what a sensor measures
- `Q`, `R`: process and measurement noise

### extended kalman filter (non-linear model)

**EKF equations**

Nonlinear models:

- Motion: `x_k = f(x_{k−1}, u_k) + noise`
- Measurement: `z_k = h(x_k) + noise`

**Predict**

```
x = f(x, u)              # use the real nonlinear function
F = ∂f/∂x  at x          # Jacobian of motion model
P = F P Fᵀ + Q
```

**Update**

```
H = ∂h/∂x  at x          # Jacobian of measurement model
y = z − h(x)             # innovation, using the real h
S = H P Hᵀ + R
K = P Hᵀ S⁻¹
x = x + K y
P = (I − K H) P
```

### unscented kalman filter (non-linear model)

**UKF equations**

**Predict**

```
χ  = sigma_points(x, P)
χ' = f(χᵢ, u)                         for each point
x  = Σ Wᵐ χ'ᵢ
P  = Σ Wᶜ (χ'ᵢ − x)(χ'ᵢ − x)ᵀ + Q
```

**Update**

```
𝒵ᵢ  = h(χ'ᵢ)                          predicted measurement per point
ẑ   = Σ Wᵐ 𝒵ᵢ
S   = Σ Wᶜ (𝒵ᵢ − ẑ)(𝒵ᵢ − ẑ)ᵀ + R       measurement covariance
Pxz = Σ Wᶜ (χ'ᵢ − x)(𝒵ᵢ − ẑ)ᵀ          cross-covariance
K   = Pxz S⁻¹
x   = x + K (z − ẑ)
P   = P − K S Kᵀ
```

### particle filter

**The algorithm**

You maintain `N` particles, each with a state `xᵢ` (for example position along the pipe and which pipe segment) and a weight `wᵢ`.

**1. Initialize**

- If the start is known, put all particles near it.
- If unknown, spread them over the whole map (global localization).

**2. Predict (motion update)**

Move every particle with the motion model, **adding random noise** to each:

```
xᵢ = f(xᵢ, u) + random noise
```

The noise makes particles spread out, representing growing uncertainty, the same role `Q` plays in a Kalman filter.

**3. Update (measurement weighting)**

For each particle, ask: *if the robot were here, how likely is the measurement I just got?*

```
wᵢ = wᵢ × p(z | xᵢ)
```

Then normalize so the weights sum to 1. With a Gaussian sensor model, `p(z | xᵢ) ∝ exp(−(z − h(xᵢ))² / 2R)`.

**4. Resample**

Draw `N` new particles in proportion to their weights. High-weight particles get copied several times, and low-weight particles disappear. Reset all weights to `1/N`.

**5. Estimate**

Report the weighted mean, or for multi-hypothesis cases, the densest cluster or the highest-weight particle.

Repeat predict → update → resample.