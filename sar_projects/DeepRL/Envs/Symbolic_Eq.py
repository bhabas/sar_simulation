from sympy import symbols, cos, sin, Matrix, pprint, simplify

# Define the angle symbols
phi, theta, beta, gamma = symbols('phi theta beta gamma')

# Define the first rotation matrix (for example, a rotation around the z-axis)
R_BW = Matrix([
    [ cos(phi), sin(phi)],
    [-sin(phi), cos(phi)],
])

R_PW = Matrix([
    [-cos(theta), sin(theta)],
    [-sin(theta),-cos(theta)],
])

R_BetaP = Matrix([
    [-cos(beta), sin(beta)],
    [-sin(beta),-cos(beta)],
])

R_BBeta = Matrix([
    [-sin(gamma), cos(gamma)],
    [-cos(gamma),-sin(gamma)],
])

# Multiply the matrices
Result = R_PW*R_BetaP*R_BBeta

# Print the resulting matrix
pprint(R_BW)
print()
pprint(R_PW)
print()

pprint(R_BetaP)
print()
pprint(R_BBeta)
print()
print()
pprint(simplify(Result))
