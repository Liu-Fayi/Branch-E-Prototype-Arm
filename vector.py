class Vector:
    def __init__(self, dim):
        self.data = [0] * dim
        self.dim = dim
    
    def __getitem__(self, key):
        return self.data[key]
    
    def __setitem__(self, key, value):
        self.data[key] = value

    def get_list(self):
        return self.data
    
    def set_list(self, data):
        if len(data) != self.dim:
            raise ValueError("Dimensions must match")
        self.data = data

    def __len__(self):
        return self.dim
    
    def __add__(self, other):
        if self.dim != other.dim:
            raise ValueError("Dimensions must match")
        result = Vector(self.dim)
        for i in range(self.dim):
            result[i] = self[i] + other[i]
        return result
    
    def __sub__(self, other):
        if self.dim != other.dim:
            raise ValueError("Dimensions must match")
        result = Vector(self.dim)
        for i in range(self.dim):
            result[i] = self[i] - other[i]
        return result
    
    def __mul__(self, scalar):
        result = Vector(self.dim)
        for i in range(self.dim):
            result[i] = self[i] * scalar
        return result
    
    def __rmul__(self, scalar):
        return self.__mul__(scalar)
    
    def __truediv__(self, scalar):
        result = Vector(self.dim)
        for i in range(self.dim):
            result[i] = self[i] / scalar
        return result
    
    def dot(self, other):
        if self.dim != other.dim:
            raise ValueError("Dimensions must match")
        result = 0
        for i in range(self.dim):
            result += self[i] * other[i]
        return result
    
    def cross(self, other):
        if self.dim != 3 or other.dim != 3:
            raise ValueError("Cross product is only defined for 3D vectors")
        result = Vector(3)
        result[0] = self[1] * other[2] - self[2] * other[1]
        result[1] = self[2] * other[0] - self[0] * other[2]
        result[2] = self[0] * other[1] - self[1] * other[0]
        return result
    

