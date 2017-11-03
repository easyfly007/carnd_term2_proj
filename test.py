import math
def f(mu, sigma2, x):
	return 1./math.sqrt(2.*math.pi * sigma2) * math.exp(-0.5 * (x-mu)**2 / sigma2)
print(f(10., 4, 10.))