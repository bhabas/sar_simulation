from sympy import symbols, cos, sin, Matrix, pprint, simplify
from sympy import *

# Define the angle symbols
phi, theta, beta_1, beta_2, gamma = symbols('phi theta beta_1 beta_2 gamma')
PD, L, gamma, a = symbols('PD L gamma a')


equation1 = Eq(sqrt(PD**2 + L**2 - 2*PD*L*cos(pi/2-gamma)),a)
equation2 = acos((L**2 + a**2 - PD**2)/(2*L*a))


solution_for_a = solve(equation1, a)[0]
substituted_eq2 = equation2.subs(a, solution_for_a)


pprint(factor(equation1))

# # Define the first rotation matrix (for example, a rotation around the z-axis)
# R_P_W = Matrix([
#     [ cos(theta), sin(theta)],
#     [-sin(theta), cos(theta)],
# ])

# R_B_W = Matrix([
#     [ cos(phi), sin(phi)],
#     [-sin(phi), cos(phi)],
# ])

# print("Phi_rel Rotation Matrix:")
# pprint(simplify(R_B_W.T*R_P_W))
# print("\n\n\n")
