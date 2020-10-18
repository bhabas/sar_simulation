import numpy as np

# Seed the random number generator for reproducibility
np.random.seed(0)


# Values sourced and units converted from https://wiki.bitcraze.io/misc:investigations:thrust
omega = np.array([0,469.668101711674,792.728546255824,981.642984491691,1139.8745344775,1285.64443360406,1416.02052872804,1538.43792246292,1667.55738052546,1798.45707442504,1903.7004283203,2031.24909005604,2150.83905040269,2271.58092805566,2366.45702619407,2500.91719176771])
thrust = np.array([0,0.015696,0.047088,0.077499,0.106929,0.136359,0.169713,0.20601,0.239364,0.280566,0.321768,0.365913,0.409077,0.45126,0.509139,0.567999])

y_data = thrust
x_data = omega

# And plot it
import matplotlib.pyplot as plt


from scipy import optimize

def test_func(x, a):
    return a *x*x

params, params_covariance = optimize.curve_fit(test_func, x_data, y_data)
print(params)

plt.figure(figsize=(6, 4))
plt.scatter(x_data, y_data, label='Data')
plt.plot(x_data, test_func(x_data, params[0]),
         label='Fitted function')

plt.legend(loc='best')

plt.show()