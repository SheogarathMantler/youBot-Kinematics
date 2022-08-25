import numpy as np
import math as m
import vg
"""
DH parameters:
a1 = 33,  alpha1 = pi/2, d1 = 147, theta1
a2 = 155, alpha2 = 0,    d2 = 0,   theta2
a3 = 135, alpha3 = 0,    d3 = 0,   theta3
a4 = 0,   alpha4 = pi/2, d4 = 0,   theta4
a5 = 0,   alpha5 = 0,    d5 = 217, theta5
"""
a1, alpha1, d1, offset1 = 33, m.pi/2, -147, 0
a2, alpha2, d2, offset2 = 155, 0,      0,   -m.pi/2
a3, alpha3, d3, offset3 = 135, 0,      0,   0
a4, alpha4, d4, offset4 = 0,   m.pi/2, 0,   m.pi/2
a5, alpha5, d5, offset5 = 0,   m.pi,   217,    m.pi/2

# function for matrix Ai
def singleLinkTransformMatrix(a, alpha, d, theta, offset):
    theta = theta + offset
    return np.array([[m.cos(theta), -m.sin(theta)*m.cos(alpha), m.sin(theta)*m.sin(alpha), a*m.cos(theta)],
                     [m.sin(theta), m.cos(theta)*m.cos(alpha), -m.cos(theta)*m.sin(alpha), a*m.sin(theta)],
                     [0,            m.sin(alpha),               m.cos(alpha),              d],
                     [0,            0,                          0,                         1]])

# function to get vector a from translation matrix T
def gripperVectorFromT(T):
    ax = T[0][2]
    ay = T[1][2]
    az = T[2][2]
    return np.array([ax, ay, az])

# function to get position vector from translation matrix T
def getP(T):
    px = T[0][3]
    py = T[1][3]
    pz = T[2][3]
    return np.array([px, py, pz])
def getR(T):
  R = np.array(T[0:3,0:3])
  return R

def test_borders(q):
  is_ok = False
  q = q*180/np.pi
  if abs(q[0]) < 169 and (-65 < q[1] < 90) and (-151 < q[2] < 146) and abs(q[3]) < 102 and abs(q[4]) < 167: is_ok = True
  return is_ok

# function for matrix T = A1*A2*A3*A4*A5
def forwardKinematics(theta1, theta2, theta3, theta4, theta5):
    print('FORWARD KINEMATICS')
    print('given thetas: ', theta1**180/m.pi, theta2*180/m.pi, theta3*180/m.pi, theta4*180/m.pi, theta5*180/m.pi)
    if test_borders(np.array([theta1, theta2, theta3, theta4, theta5])):
      A1 = singleLinkTransformMatrix(a1, alpha1, d1, theta1, offset1)
      A2 = singleLinkTransformMatrix(a2, alpha2, d2, theta2, offset2)
      A3 = singleLinkTransformMatrix(a3, alpha3, d3, theta3, offset3)
      A4 = singleLinkTransformMatrix(a4, alpha4, d4, theta4, offset4)
      A5 = singleLinkTransformMatrix(a5, alpha5, d5, theta5, offset5)
      T = A1.dot(A2).dot(A3).dot(A4).dot(A5)
      print("p: ", getP(T))
      print("euler angles: ", Rotation.from_matrix(getR(T)).as_euler('xyz', degrees=True))
      return T
    else:
      return 'does not fit borders'  

def inversePositionKinematics(x, y, z, phi, theta1, theta5, z5): 
    print('given data: x:', x, 'y: ', y, 'z: ', z, 'phi: ', phi*180/m.pi,'theta1: ', theta1*180/m.pi, 'theta5: ', theta5)
    x1 = m.cos(-theta1)*x - m.sin(-theta1)*y                                    # повернем все на тета1, чтобы проще работать, -тета1 потому что в другую сторону вращение 
    y1 = m.sin(-theta1)*x + m.cos(-theta1)*y
    x_axis = [m.cos(theta1), m.sin(theta1), 0]                                  #вычисляем координаты новой оси х как первый столбец матрицы поворота
    print('x_axis: ', x_axis, 'x1: ', x1,'y1: ', y1)
    if np.dot(z5, x_axis) < 0:                                                  # 
      xw = x1 - d5 * m.cos(phi)
    else:
      xw = x1 + d5 * m.cos(phi)  
    zw = z + d5 * m.sin(phi) - d1                                               # + потому что zw и z < 0, ведь координаты инвертированы
    print('xw: ', xw, 'zw: ', zw)
    d = m.sqrt(zw**2 + (xw - a1)**2)
    print('d: ', d)
    ct3 = (d**2 - a2**2 - a3**2) / (2 * a2 * a3)
    print('cos(theta3)= ', ct3)
    st3 = m.sqrt(1 - ct3**2)
    theta3 = m.atan2(st3, ct3)
    alpha = m.atan2(-zw, xw - a1)                                               # -zw ведь он отрицательный
    beta = -m.atan2(a3 * st3, a2 + a3 * ct3)                                    # beta и theta3 должны быть разных знаков
    print('alpha: ', alpha*180/m.pi, 'beta: ', beta*180/m.pi)
    theta2 = m.pi/2 - alpha + beta
    if np.dot(z5, x_axis) < 0:                                                  # из решения уравнения с модулем
      theta4 = m.pi/2 - theta2 - theta3 - phi
    else:
      theta4 = phi - m.pi/2 - theta2 - theta3  
    if theta1 > m.pi: theta1 -= 2*m.pi  
    print([theta1*180/m.pi, theta2*180/m.pi, theta3*180/m.pi, theta4*180/m.pi, theta5*180/m.pi])  
    return [theta1, theta2, theta3, theta4, theta5]

def inverseKinematics(R,p):
    print('INVERSE KINEMATICS')
    print('given p: ', p)
    print('given euler angles: ', Rotation.from_matrix(R).as_euler('xyz', degrees=True))
    z5 = R[:,2]
    z0 = np.array([0, 0, 1])
    print('z5: ', z5)                                                           # z5 с системе координат x0y0z0, не забывай что z0 направлена вниз, x0 по направлению смещения в 33 мм.
    theta1 = m.atan2(p[1], p[0])
    if abs(theta1*180/m.pi)>169:
      print('WARNING не развернуться на такой угол по тета1, прибавляю пи')
      theta1 += m.pi
      if theta1 > 2*m.pi: theta1 -=2*m.pi
    print('theta1: ', theta1*180/m.pi)
    n = np.cross(p, z0)
    #if n[2]<10**(-10): n[2]=0
    print('n: ', n)
    print('n * z5: ', np.dot(z5, n))
    if abs(np.dot(z5, n)) >= 10**(-3): return "error"
    ppr = p.copy()
    ppr[2] = 0
    cosphi = np.dot(-z5, ppr)/np.linalg.norm(z5)/np.linalg.norm(ppr)            # минус не уверен почему но сработало для [0 0.1 0 0 0]
    sinphi = m.sqrt(1-cosphi**2) if z5[2]>0 else -m.sqrt(1-cosphi**2)           # если z5 направлен физически вниз, то обратный к нему - вверх, и синус фи положительный
    phi = m.atan2(sinphi, cosphi)
    print('phi: ', phi*180/m.pi, cosphi, sinphi)
    R_z0 = Rotation.from_rotvec(theta1 * z0).as_matrix()
    print('R_z0: \n', R_z0)
    y1 = np.dot(R_z0, np.array([0, 1, 0]))
    print('y1: ', y1)
    R_y1 = Rotation.from_rotvec((phi-m.pi/2) * y1).as_matrix()
    print('R_y1: \n', R_y1)
    trace = np.trace(np.transpose(R_y1).dot(np.transpose(R_z0)).dot(R))
    print('trace: ', trace)
    costheta5 = (trace - 1)/2
    theta5 = m.acos(costheta5)-m.pi/2
    print('theta5: ', theta5)
    try:
      print('\n trying with theta1 =', theta1*180/m.pi)
      return inversePositionKinematics(p[0],p[1], p[2], phi, theta1, theta5, z5)
    except:
      print('\n trying with theta1 + pi =', theta1*180/m.pi + m.pi*180/m.pi)
      return inversePositionKinematics(p[0],p[1], p[2], phi, theta1+m.pi, theta5, z5) 

