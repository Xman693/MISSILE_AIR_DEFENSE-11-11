
import numpy as np
import matplotlib.pyplot as plt

def integrate(gamma_cand_deg=0.0):

    # Parameters
    dt = 0.1
    t_final = 3.0
    k = 3.0
    g = 9.81
    v = 1000.0

    gamma_cand = np.deg2rad(gamma_cand_deg)
    gamma = np.deg2rad(30.0)
    alpha = 0.0
    theta = gamma + alpha

    # Time
    time = np.arange(0, t_final + dt, dt)

    # Logs
    gamma_log = []
    theta_log = []
    alpha_log = []

    for t in time:

        # Guidance command
        gamma_dot_cmd = -k * (gamma - gamma_cand)

        # Dynamics
        alpha_dot = -k * alpha + q

        gamma_dot = gamma_dot_cmd - g/v * np.cos(gamma)

        theta_dot = (
            gamma_dot_cmd
            - g/v * np.cos(gamma)
            - k * alpha
        )

        # Log states (degrees)
        gamma_log.append(np.rad2deg(gamma))
        theta_log.append(np.rad2deg(theta))
        alpha_log.append(np.rad2deg(alpha))

        # Euler integration
        gamma += gamma_dot * dt
        alpha += alpha_dot * dt
        theta += theta_dot * dt

    return time, theta_log, gamma_log, alpha_log


# Run simulation
time, theta, gamma, alpha = integrate(gamma_cand_deg=45)


# Plot
fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

axes[0].plot(time, theta)
axes[0].set_ylabel('Theta (deg)')
axes[0].set_title('Pitch Angle')
axes[0].grid(True)

axes[1].plot(time, gamma)
axes[1].set_ylabel('Gamma (deg)')
axes[1].set_title('Flight-Path Angle')
axes[1].grid(True)

axes[2].plot(time, alpha)
axes[2].set_ylabel('Alpha (deg)')
axes[2].set_title('Angle of Attack')
axes[2].set_xlabel('Time (s)')
axes[2].grid(True)

plt.tight_layout()
plt.show()
