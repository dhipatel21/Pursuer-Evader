import numpy
from numpy import random
from Vector2D import *

class Pursuit:
    def __init__(self, pursuer_position, speed, evader_position, evader_speed, upper_bounds, lower_bounds):
        self.pursuer_position : Vector2D = pursuer_position
        self.speed = speed

        self.evader_position = evader_position
        self.evader_speed = evader_speed

        self.upper_bounds : Vector2D = upper_bounds
        self.lower_bounds : Vector2D = lower_bounds

        self.evader_position_memory : list[Vector2D] = []  
        self.evader_direction_memory : list[Vector2D] = []  
        # self.evader_position_memory : list[Vector2D] = []  
        self.pursuer_position_memory : list[Vector2D] = []
        # self.pursuer_position_memory : list[Vector2D] = []
        self.evader_direction : Vector2D = (self.evader_position - self.pursuer_position).normalize()
        self.memory_size = 5  # The size of the memory for past positions

    def populate_initial_memory(self, slam_position, pic_position):
        self.pursuer_position_memory.append(slam_position)
        self.evader_position_memory.append(pic_position)

        return len(self.pursuer_position_memory)

    def calculate_moving_average_direction(self, new_direction):
        """
        Append the new direction to the list and calculate the moving average of the
        most recent directions up to the memory size.
        """
        self.evader_position_memory.append(new_direction)
        
        # Keep only the last 'memory_size' number of directions
        self.evader_position_memory = self.evader_position_memory[-self.memory_size:]
        
        # Calculate the moving average of directions
        avg_x = sum(d.x for d in self.evader_position_memory) / len(self.evader_position_memory)
        avg_y = sum(d.y for d in self.evader_position_memory) / len(self.evader_position_memory)

        # Return the moving average as a new Vector2D object
        return Vector2D(avg_x, avg_y)
    
    def predict_velocity(self, prediction_time=1.0):
        """
        Use the stored past positions of the evader to predict its current velocity.
        """
        # Ensure there are enough points to make a prediction
        if len(self.evader_direction_memory) < 2:
            return Vector2D(0, 0)

        # Use the last two positions to calculate the velocity vector
        last_direction = self.evader_direction_memory[-1]
        second_last_direction = self.evader_direction_memory[-2]

        evader_velocity_x = (last_direction.x - second_last_direction.x) / prediction_time
        evader_velocity_y = (last_direction.y - second_last_direction.y) / prediction_time

        velocity = Vector2D(evader_velocity_x, evader_velocity_y).normalize() * self.evader_speed
        
        return velocity
    
    def predict_future_position(self, prediction_time=1.0):
        """
        Use the stored past positions of the evader to predict its future position.
        """
        # Ensure there are enough points to make a prediction
        if len(self.evader_position_memory) < 2:
            return self.evader_position

        # Use the last two positions to calculate the velocity vector
        last_position = self.evader_position_memory[-1]
        velocity = self.predict_velocity()

        # Use the velocity to predict the future position
        future_position = Vector2D(
            last_position.x + velocity.x * prediction_time,
            last_position.y + velocity.y * prediction_time,
        )

        # future_position = future_position.normalize() * 10
        
        return future_position
    
    def update_positions(self, new_evader_direction, new_pursuer_position):
        # Assume the new direction is a unit vector (magnitude of 1)
        self.pursuer_position = new_pursuer_position

        self.evader_direction_memory.append(new_evader_direction)
        self.pursuer_position_memory.append(self.pursuer_position)

        new_evader_position = self.evader_position + new_evader_direction * self.evader_speed
        self.evader_position_memory.append(new_evader_position)
        
        # Keep only the most recent `memory_size` positions
        self.evader_position_memory = self.evader_position_memory[-self.memory_size:]
        
        # Update the evader's actual position
        self.evader_position = new_evader_position
    
    def calculate_moving_average_direction(self, new_direction):
        self.evader_position_memory.append(new_direction)
        
        # Keep only the last 'memory_size' number of directions
        self.evader_position_memory = self.evader_position_memory[-self.memory_size:]
        
        # Calculate the moving average of directions
        avg_x = sum(d.x for d in self.evader_position_memory) / len(self.evader_position_memory)
        avg_y = sum(d.y for d in self.evader_position_memory) / len(self.evader_position_memory)

        # Return the moving average as a new Vector2D object
        return Vector2D(avg_x, avg_y)

    def update_pursuer_stern_chase(self, pursuer_position, evader_direction, prediction_time=1.0):
        """
        Predict the future position of the evader and move the pursuer toward that position.
        `evader_direction` should be a normalized Vector2D (unit vector).
        `prediction_time` is the amount of time into the future the prediction should be considered.
        """        
        self.update_positions(evader_direction, pursuer_position)
        future_evader_position = self.pursuer_position + evader_direction * prediction_time * self.speed
        
        print(future_evader_position.magnitude())
        future_evader_position.saturate(self.upper_bounds, self.lower_bounds)
        
        # Return the pursuer's next waypoint
        return future_evader_position
    
    def update_pursuer_smooth_chase(self, evader_direction, prediction_time=1.0):
        """
        Predict the future position of the evader and move the pursuer toward that position.
        `evader_direction` should be a normalized Vector2D (unit vector).
        `prediction_time` is the amount of time into the future the prediction should be considered.
        """

        current_direction = self.calculate_moving_average_direction(evader_direction)
        
        self.pursuer_position = self.pursuer_position + current_direction
        self.pursuer_position.saturate(self.upper_bounds, self.lower_bounds)

        # Return the pursuer's next waypoint
        return self.pursuer_position
    
    def update_pursuer_converging_chase(self, pursuer_position, evader_direction, prediction_time=5.0):
        # Update the evader's position and history
        self.update_positions(evader_direction, pursuer_position)
        
        # Predict the future position of the evader
        future_evader_position = self.predict_future_position(prediction_time=prediction_time)

        # Calculate the direction to the future position of the evader
        to_future_evader = Vector2D(
            future_evader_position.x - self.pursuer_position.x,
            future_evader_position.y - self.pursuer_position.y,
        ).normalize()

        # Update the pursuer's position towards the predicted future position of the evader
        ideal_pursuer = self.pursuer_position + to_future_evader * self.evader_speed
        print(ideal_pursuer.magnitude())
        
        ideal_pursuer.saturate(self.upper_bounds, self.lower_bounds)
        
        # Return the pursuer's next waypoint
        return ideal_pursuer

    def intercept_vector(self, evader_direction):
        velocity = self.predict_velocity().normalize()
        self.evader_direction = evader_direction
        self.evader_direction.normalize()

        if (velocity.x < 0 and velocity.y < 0):
            return
        elif (velocity.x < 0 and velocity.y > 0):
            return
        elif (velocity.x < 0 and velocity.y == 0):
            return
        elif (velocity.x > 0 and velocity.y < 0):
            return
        elif (velocity.x > 0 and velocity.y > 0):
            return
        elif (velocity.x > 0 and velocity.y == 0):
            return
        elif (velocity.x == 0 and velocity.y < 0):
            return
        elif (velocity.x == 0 and velocity.y > 0):
            return
        elif (velocity.x == 0 and velocity.y == 0):
            return
        else:
            return
