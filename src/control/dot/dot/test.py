from planner import Planner
import numpy as np

pl = Planner()

n = 3
m = 6

pl.n = n # number of agents
pl.m = m # number of tasks

pl.dist = 25*np.random.rand(n,m).flatten()

pl.p = 1.0
pl.pi = np.zeros((n,m)).flatten()

res = pl.optimize()

print(res.reshape(n,m))
