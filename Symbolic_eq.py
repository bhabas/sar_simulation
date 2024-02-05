from sympy import symbols, cos, sin, Matrix, pprint, simplify
from sympy import *


K_x, K_y = symbols('K_x K_y')
theta = symbols('theta')
# Define the first rotation matrix (for example, a rotation around the z-axis)
K = Matrix([
    [K_x, 0],
    [0, K_y],
])

T = Matrix([
    [ cos(theta), sin(theta)],
    [-sin(theta), cos(theta)],
])
K_prime = T.inv()*K*T
# print("Phi_rel Rotation Matrix:")
pprint(simplify(K_prime))
print("\n\n\n")


K_prime_at_45_deg = K_prime.subs(theta, pi/4)
pprint(simplify(K_prime_at_45_deg))
print("\n\n\n")

