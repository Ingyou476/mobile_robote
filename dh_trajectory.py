# -- coding: utf-8 --
"""
Programme 2 — Trajectoire : DH5→0 pour chaque point
EPISEN | Modélisation géométrique directe (DH)
Auteur : Vous

Ce script :
1) Génère une trajectoire articulaire Q(t) de N points (modifiable)
2) Pour chaque point, calcule T_5_0, (X,Y,Z) et (α,β,γ) en RPY (ZYX)
3) Enregistre les résultats dans un CSV
4) Trace la trajectoire 3D de la pince + les courbes α,β,γ (enregistre en PNG)
"""

import numpy as np
import math
import csv
from pathlib import Path
import matplotlib.pyplot as plt

# ---------- Fonctions utilitaires ----------
def dh(a, alpha, d, theta):
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,       sa,     ca,    d],
        [0,        0,      0,    1]
    ], dtype=float)

def rpy_from_R_zyx(R, eps=1e-9):
    r20 = R[2, 0]
    if abs(r20) < 1 - eps:
        beta = math.asin(-r20)
        alpha = math.atan2(R[2, 1], R[2, 2])
        gamma = math.atan2(R[1, 0], R[0, 0])
    else:
        beta = math.pi/2 if r20 < 0 else -math.pi/2
        alpha = 0.0
        gamma = math.atan2(-R[0,1], R[1,1])
    return alpha, beta, gamma

def forward_kinematics(Q_rad, L):
    """Calcule T_5_0 pour une configuration Q (rad) et longueurs L."""
    q1, q2, q3, q4, q5 = Q_rad
    dh_table = np.array([
        [0.0,    np.pi/2,  L[0], q1],
        [L[3],   0.0,      0.0,  q2],
        [0.0,    0.0,      L[1], q3],
        [0.0,    np.pi/2,  L[2], q4],
        [L[4],   0.0,      0.0,  q5]
    ], dtype=float)
    T = np.eye(4, dtype=float)
    for (a, alpha, d, theta) in dh_table:
        T = T @ dh(a, alpha, d, theta)
    return T

# ---------- Paramètres du robot ----------
L = np.array([0.1, 0.2, 0.15, 0.05, 0.05], dtype=float)

# ---------- Trajectoire (modifiable) ----------
N = 200  # nombre de points (peut être 50, 100, 200, 1000...)
t = np.linspace(0, 1, N)

# Ranges (en degrés) – faciles à changer selon le TP
Q1_deg = np.linspace(0, 45, N)
Q2_deg = np.linspace(-45, 30, N)
Q3_deg = 20 * np.sin(2*np.pi*t)  # oscillation douce
Q4_deg = np.zeros(N)             # fixe (peut varier si besoin)
Q5_deg = np.linspace(0, 30, N)

Q_deg = np.vstack([Q1_deg, Q2_deg, Q3_deg, Q4_deg, Q5_deg]).T
Q_rad = np.deg2rad(Q_deg)

# ---------- Calcul pour chaque point ----------
results = []  # chaque ligne : [i, X, Y, Z, alpha, beta, gamma, q1..q5]
for i in range(N):
    T = forward_kinematics(Q_rad[i], L)
    pos = T[:3, 3]
    R = T[:3, :3]
    alpha, beta, gamma = rpy_from_R_zyx(R)
    results.append([
        i, pos[0], pos[1], pos[2], alpha, beta, gamma,
        *Q_rad[i]  # on stocke aussi les q en rad
    ])

results = np.array(results, dtype=float)

# ---------- Sauvegarde CSV ----------
out_dir = Path("./outputs")
out_dir.mkdir(parents=True, exist_ok=True)
csv_path = out_dir / "trajectory_results.csv"
with open(csv_path, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["index", "X", "Y", "Z", "alpha(rad)", "beta(rad)", "gamma(rad)",
                     "q1(rad)", "q2(rad)", "q3(rad)", "q4(rad)", "q5(rad)"])
    writer.writerows(results.tolist())

# ---------- Tracés ----------
# 3D de la position
fig1 = plt.figure()
ax = fig1.add_subplot(111, projection="3d")
ax.plot(results[:,1], results[:,2], results[:,3])
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Z [m]")
ax.set_title("Trajectoire 3D de la pince")
fig1.savefig(out_dir / "trajectory_3D.png", dpi=150)
plt.close(fig1)

# Angles α, β, γ en 2D (3 figures séparées)
fig2 = plt.figure()
plt.plot(results[:,0], results[:,4])
plt.xlabel("index")
plt.ylabel("alpha [rad]")
plt.title("Évolution de α (roll)")
fig2.savefig(out_dir / "alpha.png", dpi=150)
plt.close(fig2)

fig3 = plt.figure()
plt.plot(results[:,0], results[:,5])
plt.xlabel("index")
plt.ylabel("beta [rad]")
plt.title("Évolution de β (pitch)")
fig3.savefig(out_dir / "beta.png", dpi=150)
plt.close(fig3)

fig4 = plt.figure()
plt.plot(results[:,0], results[:,6])
plt.xlabel("index")
plt.ylabel("gamma [rad]")
plt.title("Évolution de γ (yaw)")
fig4.savefig(out_dir / "gamma.png", dpi=150)
plt.close(fig4)

print("Fichiers écrits dans:", out_dir.resolve())
print(" -", csv_path.name)
print(" - trajectory_3D.png, alpha.png, beta.png, gamma.png")
