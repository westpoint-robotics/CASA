import numpy as np
from scipy.optimize import minimize
import copy

class Planner:

    def __init__(self):

        self.p_ = 1
        self.dist_ = np.array([])
        self.conn_ = np.array([])
        self.pi_ = np.array([])

        self.n_ = 1
        self.m_ = 1

        
    @property
    def p(self):
        return self.p_


    @p.setter
    def p(self, p):
        self.p_ = p


    @property
    def dist(self):
        return self.dist_


    @dist.setter
    def dist(self, d):
        self.dist_ = d


    @property
    def conn(self):
        return self.conn_


    @conn.setter
    def conn(self, c):
        self.conn_ = c


    @property
    def pi(self):
        return self.pi_


    @pi.setter
    def pi(self, p):
        self.pi_ = p


    @property
    def n(self):
        return self.n_


    @n.setter
    def n(self, n):
        self.n_ = n


    @property
    def m(self):
        return self.m_


    @m.setter
    def m(self, m):
        self.m_ = m


    def setRowConstraint(self, start, stop):
        return lambda u: np.sum(u[start:stop]) - self.p_

    
    def setColumnConstraint(self, i):
        return lambda u: self.p_ - np.sum(u.reshape(self.n_,self.m_)[:,i])

    
    def optimize(self):

        func = lambda u: np.matmul(self.dist, u)

        cons = [{'type': 'eq', 'fun': lambda u: self.n_ - np.sum(u)}]

        start = 0
        stop = self.m_
        
        for i in range(self.n_):
            x = self.setRowConstraint(start,stop)
            cons.append( {'type': 'eq', 'fun': x} )
            start = stop
            stop = stop + self.m_

        for i in range(self.m_):
            y = self.setColumnConstraint(i)
            cons.append( {'type': 'ineq', 'fun': y} )
            
        bnds = np.zeros((len(self.pi_),2))
        bnds[:,1] = self.p_
        
        res = minimize(func,
                       self.pi_,
                       method = 'SLSQP',
                       bounds = bnds,
                       constraints = cons)

        return res.x.reshape(self.n_, self.m_)
        
