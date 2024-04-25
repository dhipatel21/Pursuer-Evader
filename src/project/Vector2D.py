class Vector2D:
    def __init__(self, x, y):
        self.x : float = x
        self.y : float = y

    def __add__(self, other):
        return Vector2D(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other):
        return Vector2D(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar):
        return Vector2D(self.x * scalar, self.y * scalar)

    def __repr__(self):
        return f"({self.x}, {self.y})"
    
    def normalize(self):
        if (self.magnitude() != 0):
            return self * (1.0 / self.magnitude())
        else:
            return Vector2D(0, 0)        

    def magnitude(self):
        return (self.x ** 2 + self.y ** 2) ** 0.5
    
    def saturate(self, upper_limits, lower_limits):
        if (self.x > upper_limits.x) : self.x = upper_limits.x
        if (self.y > upper_limits.y) : self.y = upper_limits.y
        if (self.x < lower_limits.x) : self.x = lower_limits.x
        if (self.y < lower_limits.y) : self.y = lower_limits.y

        return self
    