from numpy import random
from Vector2D import Vector2D

class Evasion:
    def __init__(self, evader_position, speed, upper_bounds, lower_bounds):
        self.evader_position : Vector2D = evader_position
        self.speed = speed

        self.upper_bounds : Vector2D = upper_bounds
        self.lower_bounds : Vector2D = lower_bounds

        self.direction_state = 0

    def update_evader_random(self):
        """
        Move randomly according to the evader speed.
        """
        delta = Vector2D((random.random() - 0.5) * 2, (random.random() - 0.5) * 2)
        self.evader_position = self.evader_position + delta
        self.evader_position = self.evader_position.saturate(self.upper_bounds, self.lower_bounds)

        # Return the pursuer's next waypoint
        return self.evader_position
    
    def update_evader_CW(self, leg_done):
        """
        Move to the edge of the arena and then procees clockwise.
        """
        if (leg_done):
            self.direction_state += 1
        if (self.direction_state == 4):
            self.direction_state = 0

        if ((self.evader_position.x == self.upper_bounds.x and self.evader_position.y != self.lower_bounds.y)) or (self.direction_state == 0):
            self.evader_position.y -= self.speed
        elif ((self.evader_position.y == self.lower_bounds.y and self.evader_position.x != self.lower_bounds.x)) or (self.direction_state == 1):
            self.evader_position.x -= self.speed
        elif ((self.evader_position.x == self.lower_bounds.x and self.evader_position.y != self.upper_bounds.y)) or (self.direction_state == 2):
            self.evader_position.y += self.speed
        elif ((self.evader_position.y == self.upper_bounds.y and self.evader_position.x != self.upper_bounds.x)) or (self.direction_state == 3):
            self.evader_position.x += self.speed
        else:
            self.evader_position.x += self.speed
            self.evader_position.y += self.speed

        self.evader_position = self.evader_position.saturate(self.upper_bounds, self.lower_bounds)

        # Return the pursuer's next waypoint
        return self.evader_position
    
    def update_evader_CCW(self, leg_done):
        """
        Move to the endge of the arena and then proceed counter-clockwise.
        """
        if (leg_done):
            self.direction_state += 1
        if (self.direction_state == 4):
            self.direction_state = 0

        if ((self.evader_position.x == self.upper_bounds.x and self.evader_position.y != self.upper_bounds.y)) or (self.direction_state == 0):
            self.evader_position.y += self.speed
        elif ((self.evader_position.y == self.lower_bounds.y and self.evader_position.x != self.upper_bounds.x)) or (self.direction_state == 1):
            self.evader_position.x += self.speed
        elif ((self.evader_position.x == self.lower_bounds.x and self.evader_position.y != self.lower_bounds.y)) or (self.direction_state == 2):
            self.evader_position.y -= self.speed
        elif ((self.evader_position.y == self.upper_bounds.y and self.evader_position.x != self.lower_bounds.x)) or (self.direction_state == 3):
            self.evader_position.x -= self.speed
        else:
            self.evader_position.x += self.speed
            self.evader_position.y += self.speed

        self.evader_position = self.evader_position.saturate(self.upper_bounds, self.lower_bounds)

        # Return the pursuer's next waypoint
        return self.evader_position
