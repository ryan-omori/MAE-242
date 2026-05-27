import numpy as np
import matplotlib.pyplot as plt

# System matrices
A = np.array([
    [1,    0.4,  0,    0  ],
    [-0.6, 1,    0.4,  0  ],
    [0,    0.4,  1,   -0.6],
    [0,    0,    0.4,  1  ]
])
B = np.array([[1],[0],[0],[0]], dtype=float)
C = np.array([[0, 0, 0, 1]], dtype=float)

Q = C.T @ C   # 4x4
R = np.array([[1.0]])

T_max = 15
spectral_radii = []

P = Q.copy()  # P_1 = Q (set P_1 = Q as instructed)

for T in range(1, T_max + 1):
    # Compute K_T
    BtP = B.T @ P
    K = -np.linalg.inv(R + BtP @ B) @ BtP @ A
    
    # Closed-loop matrix
    Acl = A + B @ K
    
    # Check stability
    eigvals = np.linalg.eigvals(Acl)
    rho = np.max(np.abs(eigvals))
    spectral_radii.append(rho)
    print(f"T={T:2d}: rho(A+BK_T) = {rho:.4f}, Stable: {rho < 1}")
    
    # Update P for next iteration
    P = Q + K.T @ R @ K + Acl.T @ P @ Acl

# Plot
plt.figure(figsize=(8, 4))
plt.plot(range(1, T_max+1), spectral_radii, 'bo-')
plt.axhline(1.0, color='r', linestyle='--', label='Stability boundary')
plt.xlabel('Horizon T')
plt.ylabel('Spectral radius ρ(A + BK_T)')
plt.title('Spectral Radius vs Horizon T')
plt.legend()
plt.grid(True)
plt.xticks(range(1, T_max+1))
plt.tight_layout()
plt.show()

## Repeat for part f
rho_vals = np.arange(1.0, 0.0, -0.1)
Ts_list = []

for rho_scale in rho_vals:
    Q_scaled = rho_scale * Q
    P = Q_scaled.copy()
    Ts = None
    for T in range(1, T_max + 1):
        BtP = B.T @ P
        K = -np.linalg.inv(R + BtP @ B) @ BtP @ A
        Acl = A + B @ K
        sr = np.max(np.abs(np.linalg.eigvals(Acl)))
        if sr < 1 and Ts is None:
            Ts = T
        P = Q_scaled + K.T @ R @ K + Acl.T @ P @ Acl
    Ts_list.append(Ts if Ts else np.nan)

plt.figure(figsize=(7, 4))
plt.plot(rho_vals, Ts_list, 'rs-')
plt.xlabel('ρ (scaling of Q)')
plt.ylabel('T_s(ρ)')
plt.title('Critical Horizon T_s vs Q scaling ρ')
plt.grid(True)
plt.tight_layout()
plt.show()