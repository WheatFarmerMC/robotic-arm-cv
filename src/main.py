from src.simulation import Arm2DSimulation
from src.kinematics import Arm2D
import math

theta1 = math.radians(45)
theta2 = math.radians(30)
theta3 = math.radians(15)


def main():
    link1_length = 5.0
    link2_length = 3.0
    link3_length = 1.0

    arm_simulation = Arm2DSimulation(link1_length, link2_length, link3_length)
    arm_logic = Arm2D(link1_length, link2_length, link3_length)

    def handle_click(x, y):
        global theta1, theta2, theta3
        theta1, theta2, theta3 = arm_logic.inverse_kinematics_jacobian_pinv(
            x, y, theta_init=(theta1, theta2, theta3)
        )
        arm_simulation.set_target_angles(theta1, theta2, theta3)

    arm_simulation.set_click_callback(handle_click)

    arm_simulation.set_pose_immediately(theta1, theta2, theta3)
    arm_simulation.show()


if __name__ == "__main__":
    main()