import math

# Joint limits (radians) — tune these to your physical/desired range of motion
THETA1_MIN = -math.radians(0)
THETA1_MAX = math.radians(180)
THETA2_MIN = -math.radians(150)
THETA2_MAX = math.radians(150)
THETA3_MIN = -math.radians(150)
THETA3_MAX = math.radians(150)

# Which elbow solution to prefer: 'down' bends link2 below link1, 'up' bends it above
ELBOW_CONFIG = 'down'