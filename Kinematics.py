import numpy as np
import math as m
"""
DH parameters:
a1 = 33,  alpha1 = pi/2, d1 = 147, theta1
a2 = 155, alpha2 = 0,    d2 = 0,   theta2
a3 = 135, alpha3 = 0,    d3 = 0,   theta3
a4 = 0,   alpha4 = pi/2, d4 = 0,   theta4
a5 = 0,   alpha5 = 0,    d5 = 217, theta5
"""
a1, alpha1, d1 = 33,  m.pi/2, 147
a2, alpha2, d2 = 155, 0,      0
a3, alpha3, d3 = 135, 0,      0
a4, alpha4, d4 = 0,   m.pi/2, 0
a5, alpha5, d5 = 0,   0,      217

# function for matrix Ai
def singleLinkTransformMatrix(a, alpha, d, theta):
    return np.array([[m.cos(theta), -m.sin(theta)*m.cos(alpha), m.sin(theta)*m.sin(alpha), a*m.cos(theta)],
                     [m.sin(theta), m.cos(theta)*m.cos(alpha), -m.cos(theta)*m.sin(alpha), a*m.sin(theta)],
                     [0,            m.sin(alpha),               m.cos(alpha),              d],
                     [0,            0,                          0,                         1]])

# function to get vector a from translation matrix T
def gripperVectorFromT(T):
    ax = T[2][0]
    ay = T[2][1]
    az = T[2][2]
    return [ax, ay, az]

# function for matrix T = A1*A2*A3*A4*A5
def forwardKinematics(theta1, theta2, theta3, theta4, theta5):
    A1 = singleLinkTransformMatrix(a1, alpha1, d1, theta1)
    A2 = singleLinkTransformMatrix(a2, alpha2, d2, theta2)
    A3 = singleLinkTransformMatrix(a3, alpha3, d3, theta3)
    A4 = singleLinkTransformMatrix(a4, alpha4, d4, theta4)
    A5 = singleLinkTransformMatrix(a5, alpha5, d5, theta5)
    return A1 @ A2 @ A3 @ A4 @ A5

def inversePositionKinematics(x, y, z, phi):
    theta1 = m.atan2(y, x)
    xw = x - d5 * m.cos(phi)
    zw = z - d5 * m.sin(phi)
    d = m.sqrt(zw**2 + (xw - a1)**2)
    ct3 = (d**2 - a2**2 - a3**2) / (2 * a2 * a3)
    st3 = m.sqrt(1 - ct3**2)
    theta3 = m.atan2(st3, ct3)
    alpha = m.atan2(zw, xw - a1)
    beta = m.atan2(a3 * st3, a2 + a3 * ct3)
    theta2 = alpha - beta
    theta4 = theta2 - theta3 + phi
    return [theta1, theta2, theta3, theta4]
print(forwardKinematics(0,0,0,0,0))

