import vector

class Matrix:
    def __init__(self, rows, cols):
        self.data = [[0] * cols for i in range(rows)]
        self.rows = rows
        self.cols = cols

    def get_index(self, row, col):
        return self.data[row][col]
    
    def set_index(self, row, col, value):
        self.data[row][col] = value

    def get_list(self):
        return self.data
    
    def set_list(self, data):
        '''Set the data of the matrix to the given list of lists
        with inner lists representing rows'''
        if len(data) != self.rows or len(data[0]) != self.cols:
            raise ValueError("Dimensions must match")
        self.data = data

    def __add__(self, other):
        if self.rows != other.rows or self.cols != other.cols:
            raise ValueError("Dimensions must match")
        result = Matrix(self.rows, self.cols)
        for i in range(self.rows):
            for j in range(self.cols):
                result.set_index(i, j, self.get_index(i, j) + other.get_index(i, j))
        return result
    
    def __sub__(self, other):
        if self.rows != other.rows or self.cols != other.cols:
            raise ValueError("Dimensions must match")
        result = Matrix(self.rows, self.cols)
        for i in range(self.rows):
            for j in range(self.cols):
                result.set_index(i, j, self.get_index(i, j) - other.get_index(i, j))
        return result
    
    def __mul__(self, scalar):
        result = Matrix(self.rows, self.cols)
        for i in range(self.rows):
            for j in range(self.cols):
                result.set_index(i, j, self.get_index(i, j) * scalar)
        return result
    
    def __rmul__(self, scalar):
        return self.__mul__(scalar)
    
    def __truediv__(self, scalar):
        result = Matrix(self.rows, self.cols)
        for i in range(self.rows):
            for j in range(self.cols):
                result.set_index(i, j, self.get_index(i, j) / scalar)
        return result
    
    def __matmul__(self, other):
        if self.cols != other.rows:
            raise ValueError("Dimensions must match")
        result = Matrix(self.rows, other.cols)
        for i in range(self.rows):
            for j in range(other.cols):
                result.set_index(i, j, vector.Vector(self.cols).dot(vector.Vector([other.get_index(k, j) for k in range(other.rows)])))
        return result
    
    def vecmul(self, vec):
        if self.cols != len(vec):
            raise ValueError("Dimensions must match")
        result = vector.Vector(self.rows)
        for i in range(self.rows):
            result[i] = vector.Vector(self.cols).dot(vec)
        return result