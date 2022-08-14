import math as m
l0, l1, l2 = 1, 2, 2
# 0 < theta1 < 2pi

def forward_kinematics(theta1, theta2, theta3):
    pz = l1*m.cos(theta2)+l2*m.cos(theta2 + theta3)
    l = l0 + l1*m.sin(theta2)+l2*m.sin(theta2 + theta3)
    print('l = ', l)
    px = l*m.cos(theta1)
    py = l*m.sin(theta1)
    print('px = ',px, 'py = ', py, 'pz = ', pz, '\n')
    return [px, py, pz]

def inverse_kinematics(px, py, pz):
    print('inverse kinematics \n')
    theta1 = m.atan2(py, px)
    l = m.sqrt(px**2 + py**2)
    print('l = ', l)
    costheta3 = ((l-l0)**2 + pz**2 - l1**2 - l2**2) / (2*l1*l2)
    theta3 = m.acos(costheta3)
    alpha = m.atan2(pz, l-l0)
    cosbeta = -(l2**2-l1**2-(l-l0)**2-pz**2) / (2*l1*m.sqrt((l-l0)**2 + pz**2))
    beta = m.acos(cosbeta)
    theta2 = m.pi/2 - alpha - beta
    print('alpha = ', alpha, 'cosbeta = ', cosbeta, 'beta = ', beta)
    return [theta1, theta2, theta3]

def inverse_kinematics1(px, py, pz):
    print('inverse kinematics \n')
    theta1 = m.atan2(py, px)
    l = m.sqrt(px**2 + py**2)
    print('l = ', l)
    costheta3 = ((l-l0)**2 + pz**2 - l1**2 - l2**2) / (2*l1*l2)
    theta3 = m.acos(costheta3)
    alpha = m.atan2(pz, l-l0)
    #cosbeta = costheta3 * l2 / m.sqrt((l-l0)**2 + pz**2) + l1**2 / (l1 * m.sqrt((l-l0)**2 + pz**2))
    cosbeta = (costheta3 * l2 + l1) / m.sqrt((l-l0)**2 + pz**2)
    beta = m.acos(cosbeta)
    theta2 = m.pi/2 - alpha - beta
    print('alpha = ', alpha, 'cosbeta = ', cosbeta, 'beta = ', beta)
    return [theta1, theta2, theta3]
fk = forward_kinematics(1, 1, 1)
print(inverse_kinematics1(fk[0], fk[1], fk[2]))