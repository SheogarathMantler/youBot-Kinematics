import numpy as np
import math as m
from scipy.spatial.transform import Rotation

a1, alpha1, d1, offset1 = 33, m.pi / 2, -147, 0
a2, alpha2, d2, offset2 = 155, 0, 0, -m.pi / 2
a3, alpha3, d3, offset3 = 135, 0, 0, 0
a4, alpha4, d4, offset4 = 0, m.pi / 2, 0, m.pi / 2
a5, alpha5, d5, offset5 = 0, m.pi, 217, m.pi / 2


# function for matrix Ai
def singleLinkTransformMatrix(a, alpha, d, theta, offset):
    theta = theta + offset
    return np.array([[m.cos(theta), -m.sin(theta) * m.cos(alpha), m.sin(theta) * m.sin(alpha), a * m.cos(theta)],
                     [m.sin(theta), m.cos(theta) * m.cos(alpha), -m.cos(theta) * m.sin(alpha), a * m.sin(theta)],
                     [0, m.sin(alpha), m.cos(alpha), d],
                     [0, 0, 0, 1]])

# function to get position vector from translation matrix T
def getP(T):
    px = T[0][3]
    py = T[1][3]
    pz = T[2][3]
    return np.array([px, py, pz])

def getR(T):
    R = np.array(T[0:3, 0:3])
    return R

# function for matrix T = A1*A2*A3*A4*A5
def forwardKinematics(theta1, theta2, theta3, theta4, theta5):
    print('FORWARD KINEMATICS')
    print('given thetas: ', theta1 * 180 / m.pi, theta2 * 180 / m.pi, theta3 * 180 / m.pi, theta4 * 180 / m.pi,
          theta5 * 180 / m.pi)
    A1 = singleLinkTransformMatrix(a1, alpha1, d1, theta1, offset1)
    A2 = singleLinkTransformMatrix(a2, alpha2, d2, theta2, offset2)
    A3 = singleLinkTransformMatrix(a3, alpha3, d3, theta3, offset3)
    A4 = singleLinkTransformMatrix(a4, alpha4, d4, theta4, offset4)
    A5 = singleLinkTransformMatrix(a5, alpha5, d5, theta5, offset5)
    T = A1.dot(A2).dot(A3).dot(A4).dot(A5)
    print("p: ", getP(T))
    print("euler angles: ", Rotation.from_matrix(getR(T)).as_euler('xyz', degrees=True))
    return T


def getframe5(R):
    x5 = R[:, 0]
    y5 = R[:, 1]
    z5 = R[:, 2]
    return [x5, y5, z5]


def getTheta1(p, theta1_config):  # 1 конфигурация - ось x1 смотрит на точку
    theta1 = m.atan2(p[1], p[0])
    if theta1_config == 2:
        if theta1 < 0:
            theta1 += m.pi
        else:
            theta1 -= m.pi
    print('theta1: ', '%.2f' % (theta1 * 180 / m.pi))
    return theta1


def getPhi(R, p, theta1):
    z0 = np.array([0, 0, 1])
    R_z0 = Rotation.from_rotvec(-theta1 * z0).as_matrix()
    z5 = np.dot(R_z0, np.array(getframe5(R)[2]))
    cosphi = np.dot(z5, z0)
    sinphi = m.sqrt(1 - cosphi ** 2) * np.sign(-z5[0])
    print('\n z5: ', z5, '\n cos(phi): ', cosphi, 'sin(phi): ', sinphi)
    phi = m.atan2(sinphi, cosphi)
    print('phi: ', '%.2f' % (phi * 180 / m.pi))
    return phi


def phi_morphism(phi):
    phi_temp = phi
    if phi_temp == 0: return m.pi / 2
    if phi_temp == m.pi or phi_temp == -m.pi: return -m.pi / 2
    if phi_temp < 0: phi_temp = -phi_temp
    return m.pi / 2 - phi_temp


def getTheta234(p, z5, theta1, phi_old, theta5, theta3_config):
    phi = phi_morphism(phi_old)
    x_new = m.cos(-theta1) * p[0] - m.sin(-theta1) * p[1]  # повернем все на тета1, -тета1 потому что в другую сторону вращение
    y_new = m.sin(-theta1) * p[0] + m.cos(-theta1) * p[1]
    x_axis = [m.cos(theta1), m.sin(theta1), 0]  # вычисляем координаты новой оси х как первый столбец матрицы поворота
    print('x_axis: ', x_axis, 'x_new: ', '%.2f' % x_new, 'y_new: ', '%.2f' % y_new)
    if np.dot(z5, x_axis) < 0:  #
        xw = x_new - d5 * m.cos(phi)
    else:
        xw = x_new + d5 * m.cos(phi)
    zw = p[2] + d5 * m.sin(phi) - d1  # + потому что zw и z < 0, ведь координаты инвертированы
    print('xw: ', '%.2f' % xw, 'zw: ', '%.2f' % zw)
    d = m.sqrt(zw ** 2 + (xw - a1) ** 2)
    print('d: ', '%.2f' % d)
    ct3 = (d ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)
    print('cos(theta3)= ', '%.2f' % ct3)
    if theta3_config == 1:
        st3 = m.sqrt(1 - ct3 ** 2)
    else:
        st3 = -m.sqrt(1 - ct3 ** 2)
    theta3 = m.atan2(st3, ct3)
    print('theta3: ', '%.2f' % (theta3 * 180 / m.pi))
    alpha = m.atan2(-zw, xw - a1)  # -zw ведь он отрицательный
    beta = -m.atan2(a3 * st3, a2 + a3 * ct3)  # beta и theta3 должны быть разных знаков
    print('alpha: ', '%.2f' % (alpha * 180 / m.pi), 'beta: ', '%.2f' % (beta * 180 / m.pi))
    theta2 = m.pi / 2 - alpha + beta
    print('theta2: ', '%.2f' % (theta2 * 180 / m.pi))
    if np.dot(z5, x_axis) < 0:  # из решения уравнения с модулем
        theta4 = m.pi / 2 - theta2 - theta3 - phi
    else:
        theta4 = phi - m.pi / 2 - theta2 - theta3
    print('theta4: ', '%.2f' % (theta4 * 180 / m.pi))
    return [theta1, theta2, theta3, theta4, theta5]


def getTheta5(R, p, theta1, phi):
    z0 = np.array([0, 0, 1])
    R_z0 = Rotation.from_rotvec(-theta1 * z0).as_matrix()
    z5 = np.dot(R_z0, np.array(getframe5(R)[2]))
    y1 = np.dot(R_z0, np.array([0, 1, 0]))
    x1 = np.dot(R_z0, np.array([1, 0, 0]))
    R_y1 = Rotation.from_rotvec(phi * y1).as_matrix()
    R_z5 = np.transpose(R_y1).dot(np.transpose(R_z0)).dot(R)
    theta5 = m.atan2(R_z5[0, 1], R_z5[0, 0]) + m.pi / 2 - theta1  # offset
    if theta5 > m.pi: theta5 -= 2 * m.pi
    if theta5 < -m.pi: theta5 += 2 * m.pi
    print('\n theta5: ', '%.2f' % (theta5 * 180 / m.pi))
    return theta5


def inverseKinematics(T, theta1_config, theta3_config):
    z0 = np.array([0, 0, 1])
    R, p = getR(T), getP(T)
    theta_array = None

    try:
        theta1 = getTheta1(p, theta1_config)
        R_z0 = Rotation.from_rotvec(theta1 * z0).as_matrix()
        z5 = np.dot(R_z0, np.array(getframe5(R)[2]))
        phi = getPhi(R, p, theta1)
        theta5 = getTheta5(R, p, theta1, phi)
        theta_array_1 = getTheta234(p, z5, theta1, phi, theta5, theta3_config)
    except:
        print('ERROR')
    return theta_array
