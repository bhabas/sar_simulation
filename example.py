from sympy import *

# Define symbols
C = symbols('C')

d14x,d14y = symbols('d14x d14y')
d23x,d23y = symbols('d23x d23y')

M = Matrix([
    [ 1, 1, 1, 1], 
    [ -d14y, -d23y, d23y, d14y], 
    [ -d14x, d23x, d23x, -d14x], 
    [ -C, C, -C, C], 
    ])

# Compute the inverse
M_inv = M.inv()

# Print the inverse in a pretty formatted way
print("The inverse of the matrix is:")
pprint(simplify(M_inv), use_unicode=True)

