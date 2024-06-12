import math

def fmt(value):
    return "%.3f" % value

# Constants for the trajectory
period = [4, 2, 4]
radius = 1.5
timestep = 0.02
maxtime = max(period) * 3
phase = [0, 0, 0]
amp = [1, 0.4, 0.5]
center = [0, 0, -2]
yaw = 0  # Assuming constant yaw for simplicity
pitch = 0  # Assuming constant pitch
roll = 0  # Assuming constant roll

with open('FigureEight.txt', 'w') as the_file:
    t = 0
    while t <= maxtime:
        # Position calculations
        x = math.sin(t * 2 * math.pi / period[0] + phase[0]) * radius * amp[0] + center[0]
        y = math.sin(t * 2 * math.pi / period[1] + phase[1]) * radius * amp[1] + center[1]
        z = math.sin(t * 2 * math.pi / period[2] + phase[2]) * radius * amp[2] + center[2]
        
        # Velocity calculations
        vx = math.cos(t * 2 * math.pi / period[0] + phase[0]) * (2 * math.pi / period[0]) * radius * amp[0]
        vy = math.cos(t * 2 * math.pi / period[1] + phase[1]) * (2 * math.pi / period[1]) * radius * amp[1]
        vz = math.cos(t * 2 * math.pi / period[2] + phase[2]) * (2 * math.pi / period[2]) * radius * amp[2]
        
        # Angular velocities (assuming zero for simplicity)
        omega_x = 0
        omega_y = 0
        omega_z = 0
        
        # Write to file with YPR and omega
        the_file.write(fmt(t) + "," + fmt(x) + "," + fmt(y) + "," + fmt(z) + "," +
                       fmt(vx) + "," + fmt(vy) + "," + fmt(vz) + "," +
                       fmt(yaw) + "," + fmt(pitch) + "," + fmt(roll) + "," +
                       fmt(omega_x) + "," + fmt(omega_y) + "," + fmt(omega_z) + "\n")
        
        t += timestep
