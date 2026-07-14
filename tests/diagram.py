import os
import sys
import path
import math
from src.kinematics import Arm2D
from src.config import THETA1_MIN, THETA1_MAX, THETA2_MIN, THETA2_MAX, THETA3_MIN, THETA3_MAX, ELBOW_CONFIG
from matplotlib import pyplot as plt

class create_diagram:
    def __init__(self, link1_length, link2_length, link3_length=None):
        self.arm = Arm2D(link1_length, link2_length, link3_length)
        self.click_callback = None

        self.n_links = 3 if link3_length is not None else 2
        self.current_thetas = [0.0] * self.n_links
        self.target_thetas = [0.0] * self.n_links
        self.fig, self.ax = plt.subplots()

        colors = ['r', 'b', 'g']
        self.link_lines = [
            self.ax.plot([], [], color=colors[i], linewidth=5)[0]
            for i in range(self.n_links)
        ]
        self.end_effector_point, = self.ax.plot([], [], 'go')

        link_lengths = [link1_length, link2_length]
        if link3_length is not None:
            link_lengths.append(link3_length)
        limit = sum(link_lengths) + 1                     # now accounts for link3
        self.ax.set_xlim(0, limit)
        self.ax.set_ylim(0, limit)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_title('Arm')
        self.ax.grid(True)

    def _draw(self):
        vectors = self.arm.forward_kinematics(*self.current_thetas)  # per-link (dx, dy), not cumulative

        x, y = 0.0, 0.0
        for line, (dx, dy) in zip(self.link_lines, vectors):
            new_x, new_y = x + dx, y + dy
            line.set_data([x, new_x], [y, new_y])   # each link only spans its own joint-to-joint segment
            x, y = new_x, new_y

        self.end_effector_point.set_data([x], [y])

    def set_pose_immediately(self, *thetas):
        """For the very first draw — skips interpolation."""
        assert len(thetas) == self.n_links, f"Expected {self.n_links} angles, got {len(thetas)}"
        self.current_thetas = list(thetas)
        self.target_thetas = list(thetas)
        self._draw()

    def triangle(self, thetas):
        # Draw a triangle representing the arm's reachable workspace
        link_lengths = self.arm.link_lengths
        total_length = sum(link_lengths)
        print(self.arm._joint_positions(thetas)[1])
        triangle_vertices = [
            (0, 0),
            self.arm._joint_positions(thetas)[1],
            (self.end_effector_point.get_xdata()[0], self.end_effector_point.get_ydata()[0]),
            (0, 0)  # Close the triangle
        ]
        self.ax.fill([v[0] for v in triangle_vertices], [v[1] for v in triangle_vertices], alpha=0.3)
    def label(self, text, position):
        self.ax.text(position[0], position[1], text, fontsize=12, ha='center', va='center', color='black')


def main():
    diagram = create_diagram(link1_length=5.0, link2_length=3.0)
    theta1 = 45
    theta2 = 30
    diagram.set_pose_immediately(theta1, theta2)
    diagram.triangle([theta1, theta2])
    diagram.label("Joint 1", (0, 0))
    diagram.label("Joint 2", diagram.arm._joint_positions([theta1, theta2])[1])
    plt.show()

if __name__ == "__main__":
    main()