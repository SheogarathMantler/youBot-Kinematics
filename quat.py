import numpy as np
import quaternion as quat

v = [3,5,0]
axis = [4,4,1]
theta = 1.2 #radian

vector = np.array([0.] + v)
rot_axis = np.array([0.] + axis)
axis_angle = (theta*0.5) * rot_axis/np.linalg.norm(rot_axis)

vec = quat.quaternion(*v)
qlog = quat.quaternion(*axis_angle)
q = np.exp(qlog)
v_prime = q * vec * np.conjugate(q)

print(v_prime) # quaternion(0.0, 2.7491163, 4.7718093, 1.9162971)
v_prime_vec = v_prime.imag # [2.74911638 4.77180932 1.91629719] as a numpy array
