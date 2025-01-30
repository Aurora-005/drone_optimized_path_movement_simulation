import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# PID Controller Class
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.prev_error = 0
        self.integral = 0

    def update(self, setpoint, current_position):
        # Calculate error
        error = setpoint - current_position
        self.integral += error
        derivative = error - self.prev_error

        # PID formula
        control_signal = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Update previous error
        self.prev_error = error
        return control_signal

# Simulation parameters
setpoint = 0  # Target position (setpoint), along the X-axis
initial_position = 0  # Drone starts at the target position
time_steps = 100  # Number of simulation steps
dt = 0.1  # Time step (adjust for smoother animation)
wind_strength = 1.5  # Strength of wind disturbance
pid = PID(Kp=0.5, Ki=0.1, Kd=0.05)  # PID controller with arbitrary gains

# Position history for plotting
positions = [initial_position]
control_signals = []
errors = []

# Simulate drone movement with PID control
current_position = initial_position
for _ in range(time_steps):
    # Simulate wind disturbance (random)
    wind = np.random.uniform(-wind_strength, wind_strength)

    # Calculate new position after wind disturbance
    current_position += wind

    # PID control update
    control_signal = pid.update(setpoint, current_position)

    # Correct the position with the control signal
    current_position += control_signal * dt  # Apply control signal to correct position

    positions.append(current_position)
    control_signals.append(control_signal)
    errors.append(setpoint - current_position)

# Visualization using Matplotlib
fig, ax = plt.subplots()
ax.set_xlim(-5, 5)  # Set limits for the X-axis
ax.set_ylim(-5, 5)  # Set limits for the Y-axis
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_title('PID Control of Drone Position with Wind Disturbance')

# Create a square to represent the drone
drone_square = plt.Rectangle((-0.5, -0.5), 1, 1, color='blue')  # Drone is a 1x1 square
ax.add_patch(drone_square)

# Animation function to update the square's position
def update(frame):
    drone_square.set_xy((positions[frame] - 0.5, 0))  # Set the bottom-left corner of the rectangle
    return drone_square,

# Create the animation
ani = animation.FuncAnimation(fig, update, frames=range(1, time_steps+1), interval=50, blit=True)

# Display the animation
plt.show()
