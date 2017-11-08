p = [0, 1, 0, 0, 0]
world = ['green', 'red', 'green', 'green']
measurements = ['red', 'green']
pHit = 0.6
pMiss = 0.2

def dense(p, Z):
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
