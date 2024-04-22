# simulation.py

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import FancyArrowPatch
from pursuit import Pursuit
from evasion import Evasion
from Vector2D import Vector2D

# Initialize the simulation environment
pursuer_initial_position = Vector2D(0, 0)
evader_initial_position = Vector2D(10, 10)
pursuer_speed = 1.0
evader_speed = 1.0

lower_bounds : Vector2D = Vector2D(0, 0)
upper_bounds : Vector2D = Vector2D(10, 10)

pursuit_agent = Pursuit(pursuer_initial_position, pursuer_speed, evader_initial_position, evader_speed, upper_bounds, lower_bounds)
evasion_agent = Evasion(evader_initial_position, evader_speed, upper_bounds, lower_bounds)

def update_direction(pursuer : Pursuit, evader : Evasion):
    return (evader.evader_position - pursuer.pursuer_position).normalize()

evader_direction = update_direction(pursuit_agent, evasion_agent)

# Set up the figure for plotting
fig, ax = plt.subplots()
ax.set_xlim(-10, 20)
ax.set_ylim(-10, 20)

# Initialize the plot elements we want to animate
pursuer_dot, = ax.plot([], [], 'go', label='Pursuer')
evader_dot, = ax.plot([], [], 'ro', label='Evader')
est_dot, = ax.plot([], [], 'mo', label='Evader Estimate')
velocity_arrow = FancyArrowPatch((0, 0), (0, 0), color='blue', arrowstyle='->', mutation_scale=20)
ax.add_patch(velocity_arrow)
legend = ax.legend(loc='upper right')

def init():
    pursuer_dot.set_data([], [])
    evader_dot.set_data([], [])
    est_dot.set_data([], [])
    velocity_arrow.set_positions((0,0), (0,0))
    return pursuer_dot, evader_dot, est_dot, velocity_arrow

def update(frame):
    global pursuit_agent, evasion_agent, evader_direction

    # Update pursuer's position
    next_waypoint_pursuer = pursuit_agent.update_pursuer_converging_chase(evader_direction)
    next_waypoint_evader = evasion_agent.update_evader_CW()
    
    # Update plot
    pursuer_dot.set_data(next_waypoint_pursuer.x, next_waypoint_pursuer.y)
    evader_dot.set_data(next_waypoint_evader.x, next_waypoint_evader.y)
    est_dot.set_data(pursuit_agent.predict_future_position().x, pursuit_agent.predict_future_position().y)
    predicted_velocity = pursuit_agent.predict_velocity()
    arrow_start = (next_waypoint_evader.x, next_waypoint_evader.y)
    arrow_end = (arrow_start[0] + predicted_velocity.x*5, arrow_start[1] + predicted_velocity.y*5)
    velocity_arrow.set_positions(arrow_start, arrow_end)

    evader_direction = update_direction(pursuit_agent, evasion_agent)
    
    return pursuer_dot, evader_dot, est_dot, velocity_arrow

# Create the animation object
# The interval is the time between frames in milliseconds
anim = FuncAnimation(fig, update, init_func=init, frames=200, interval=500, blit=True)

# Display the animation
plt.show()