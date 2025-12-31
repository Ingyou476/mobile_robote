# -- coding: utf-8 --
"""
Programme 1 — DH5→0 pour une configuration donnée
EPISEN | Modélisation géométrique directe (DH)
Auteur : Abdel-Aziz & Florian

Ce script :
1) calcule la matrice homogène T_5_0 (repère 5 vers repère 0) pour Q = [q1..q5]
2) extrait la position (X,Y,Z) et les angles d’orientation (α,β,γ) en RPY (ZYX)
   - Convention utilisée : R = Rz(γ) * Ry(β) * Rx(α)
   - Extraction : 
        β = asin(-R[2,0])
        α = atan2(R[2,1], R[2,2])
        γ = atan2(R[1,0], R[0,0])
"""

import numpy as np
import math

# ---------- Fonctions utilitaires ----------
def dh(a, alpha, d, theta):
    """Matrice DH standard (Denavit–Hartenberg)."""
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,       sa,     ca,    d],
        [0,        0,      0,    1]
    ], dtype=float)

def rpy_from_R_zyx(R, eps=1e-9):
    """
    Convertit une matrice de rotation en angles RPY (ZYX) :
    R = Rz(γ) * Ry(β) * Rx(α)
    Retourne (α, β, γ) en radians.
    Gère le cas de verrouillage (gimbal lock).
    """
    r20 = R[2, 0]
    if abs(r20) < 1 - eps:
        beta = math.asin(-r20)
        alpha = math.atan2(R[2, 1], R[2, 2])
        gamma = math.atan2(R[1, 0], R[0, 0])
    else:
        # Gimbal lock : |r20| ~ 1
        beta = math.pi/2 if r20 < 0 else -math.pi/2
        alpha = 0.0
        gamma = math.atan2(-R[0,1], R[1,1])
    return alpha, beta, gamma

# ---------- Paramètres du robot (exemple générique 5 axes) ----------
# Longueurs des segments (m)
L = np.array([0.1, 0.2, 0.15, 0.05, 0.05], dtype=float)  # [l1, l2, l3, l4, l5]

# Configuration articulaire (degrés -> radians)
Q_deg = np.array([30, -45, 20, 0, 10], dtype=float)     # [q1, q2, q3, q4, q5] en degrés
Q = np.deg2rad(Q_deg)

# ---------- Tableau DH (a, alpha, d, theta) ----------
dh_table = np.array([
    [0.0,    np.pi/2,  L[0], Q[0]],  # i=1
    [L[3],   0.0,      0.0,  Q[1]],  # i=2
    [0.0,    0.0,      L[1], Q[2]],  # i=3
    [0.0,    np.pi/2,  L[2], Q[3]],  # i=4
    [L[4],   0.0,      0.0,  Q[4]]   # i=5
], dtype=float)

# ---------- Calcul de T_5_0 ----------
T = np.eye(4, dtype=float)
for (a, alpha, d, theta) in dh_table:
    T = T @ dh(a, alpha, d, theta)

# ---------- Extraction position + orientation ----------
pos = T[:3, 3]
R = T[:3, :3]
alpha, beta, gamma = rpy_from_R_zyx(R)

# ---------- Affichage ----------
np.set_printoptions(precision=4, suppress=True)
print("Table DH (a, alpha, d, theta):\n", np.round(dh_table, 4))
print("\nMatrice T_5_0:\n", np.round(T, 4))
print("\nPosition (X, Y, Z):", np.round(pos, 4))
print("Angles RPY (α, β, γ) [rad]:", np.round([alpha, beta, gamma], 6))
print("Angles RPY (α, β, γ) [deg]:", np.round(np.rad2deg([alpha, beta, gamma]), 4))
