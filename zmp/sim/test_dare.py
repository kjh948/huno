import numpy as np
import scipy.linalg

dt = 0.01
G = 9.8
z = 0.27
A = np.matrix([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]])
B = np.matrix([[0.0], [0.0], [1.0]])
C = np.matrix([[1.0, 0.0, -z / G]])
I = np.eye(3)
# roughly c2d
A_d = I + A*dt + A*A*dt**2/2
B_d = B*dt + A*B*dt**2/2 + A*A*B*dt**3/6
C_d = C

E_d = np.matrix([[dt], [1.0], [0.0]])
Zero = np.matrix([[0.0], [0.0], [0.0]])
Phai = np.block([[1.0, -C_d * A_d], [Zero, A_d]])
G_block = np.block([[-C_d * B_d], [B_d]])

Qm = np.zeros((4, 4))
Qm[0][0] = 1.0e+8
H = 1.0

try:
    P = scipy.linalg.solve_discrete_are(Phai, G_block, Qm, H)
    print("scipy success")
    print(P)
except Exception as e:
    print("scipy error:", e)

