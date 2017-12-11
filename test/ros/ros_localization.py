p = [0, 1, 0, 0, 0]
world = ['green', 'red', 'green', 'green']
measurements = ['red', 'green']
motions = [1,1]
pHit = 0.6
pMiss = 0.2
pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1

def sense(p, Z):
	q = []
	for i in range(len(p)):
		hit = (Z == world[i])
		q.append(p[i] * (hit * pHit + (1 - hit) * pMiss))
	s = sum(q)
	for i in range(len(q)):
		q[i] = q[i] / s
	return q

def move(p, U):
	# U is the step that the robot will move to left or right
	size = len(p)
	U = U % size
	q = p[-U:] + p[:-U]
	return q

for meas, mot in zip(measurements, motions):
	p = sense(p, meas)
	p = move(p, mot)
print(p)
