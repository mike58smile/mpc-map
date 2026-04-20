<p align="left">
  <img src="images/logo.png" height="80">
  <img src="images/fekt.png" height="80">
</p>

# MPC-MAP Assignment No. 4 - Report

**Author:** Michal Miškolci  
**Date:** 20. 4. 2026  
## Task 1

For Task 1, I loaded the `outdoor_1` map and prepared a manual trajectory from the start pose `[2,2,\pi/2]` to the goal position `[16,2]`.
The trajectory was defined by hand-selected waypoints.

![](images/createdPath.png)

I implemented the initialization procedure for the Kalman filter based on GNSS measurements.

The robot remains stationary at the beginning and collects 80 GNSS samples. From these samples, the initial position mean and covariance matrix are computed and used as the initial belief for the EKF/KF pipeline.

Measured initialization result:

```text
GNSS initialization complete.
Mean: [2.0115 1.9281]
Covariance matrix:
  0.2490   -0.0023
   -0.0023    0.1793
```

The estimated initial position is close to the true start pose, and the covariance matrix captures the GNSS uncertainty in both axes including a small correlation term.

# Task 2

I implemented EKF prediction and KF correction for localization.

The prediction step uses the nonlinear differential-drive model and its Jacobian to propagate the state and covariance. The correction step uses GNSS measurement $z=[x,y]^T$ with linear measurement model $C=[1\ 0\ 0;\ 0\ 1\ 0]$.

From simulator outputs, the corrected estimate $\mu_{corr}$ differs from the predicted estimate $\mu_{pred}$ after GNSS updates, which confirms that the correction step is active.

# Task 3

For Task 3, I set the initial EKF belief to the known pose
$\mu_0=[2,2,\pi/2]^T$ and used
$\Sigma_0=\mathbf{0}_{3\times3}$ (high certainty).

The measurement noise matrix was set from Task 1 GNSS covariance:

$$
Q = \begin{bmatrix}
0.2490 & -0.0023 \\
-0.0023 & 0.1793
\end{bmatrix}.
$$

I initialized process noise with variances $0.01$ and then tuned matrix $R$ for smoother behavior. The final tuned setting was:

```matlab
public_vars.kf.R = diag([0.0003, 0.0003, 0.0002]);
```

The controller used only the EKF estimated pose. With this setup, the robot successfully followed the trajectory and reached the goal.

Captured result:

![](images/kalmanPath1.png)

# Task 4