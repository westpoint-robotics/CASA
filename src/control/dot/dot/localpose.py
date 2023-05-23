class LocalPose():
    
    def __init__(self):
        
        self.x_ = float()
        self.y_ = float()
        self.z_ = float()
        
    @property
    def x(self):
        return self.x_
    
    @x.setter
    def x(self, x):
        self x_ = x
        
    @property
    def y(self):
        return self.y_
    
    @y.setter
    def y(self, y):
        self.y_ = y
        
    @property
    def z(self):
        return self.z_
    
    @z.setter
    def z(self, z):
        self.z_ = z
        
    def initializeFromPoint(self, point):
        self.x_ = point.x
        self.y_ = point.y
        self.z_ = point.z
