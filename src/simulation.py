from src.kinematics import Arm2D
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math


def shortest_angle_diff(target, current):
    """Signed difference target - current, wrapped to [-pi, pi] so motion always takes the short way."""
    return (target - current + math.pi) % (2 * math.pi) - math.pi


class Arm2DSimulation:
    def __init__(self, link1_length, link2_length, link3_length=None, smoothing=0.15):
        self.arm = Arm2D(link1_length, link2_length, link3_length)
        self.click_callback = None

        self.n_links = 3 if link3_length is not None else 2
        self.current_thetas = [0.0] * self.n_links
        self.target_thetas = [0.0] * self.n_links
        self.smoothing = smoothing

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
        self.ax.set_xlim(-limit, limit)
        self.ax.set_ylim(-limit, limit)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_title('Arm')
        self.ax.grid(True)

        self.fig.canvas.mpl_connect('button_press_event', self.onclick)
        self.animation = FuncAnimation(self.fig, self._animate, interval=16, cache_frame_data=False)
        self.jacobian_text = self.ax.text(
            0.02, 0.98, '', transform=self.ax.transAxes,
            fontsize=9, fontfamily='monospace', verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.85)
        )

    def set_click_callback(self, callback):
        self.click_callback = callback
    def _format_jacobian(self, J):
        rows, cols = J.shape
        lines = [f"J ({rows}x{cols}):"]
        for r in range(rows):
            row_str = "  ".join(f"{J[r, c]:6.2f}" for c in range(cols))
            lines.append(f"[{row_str}]")
        return "\n".join(lines)
    def onclick(self, event):
        if event.xdata is None or event.ydata is None:
            return
        ix, iy = event.xdata, event.ydata
        print(f'Clicked at: ({ix}, {iy})')
        if self.click_callback is not None:
            self.click_callback(ix, iy)

    def set_target_angles(self, *thetas):
        """Call this on click — sets where the arm should ease toward, not where it jumps to."""
        assert len(thetas) == self.n_links, f"Expected {self.n_links} angles, got {len(thetas)}"
        self.target_thetas = list(thetas)

    def set_pose_immediately(self, *thetas):
        """For the very first draw — skips interpolation."""
        assert len(thetas) == self.n_links, f"Expected {self.n_links} angles, got {len(thetas)}"
        self.current_thetas = list(thetas)
        self.target_thetas = list(thetas)
        self._draw()

    def _animate(self, frame):
        for i in range(self.n_links):
            diff = shortest_angle_diff(self.target_thetas[i], self.current_thetas[i])
            if abs(diff) > 1e-4:
                self.current_thetas[i] += diff * self.smoothing
        self._draw()
        return (*self.link_lines, self.end_effector_point)

    def _draw(self):
        vectors = self.arm.forward_kinematics(*self.current_thetas)  # per-link (dx, dy), not cumulative

        x, y = 0.0, 0.0
        for line, (dx, dy) in zip(self.link_lines, vectors):
            new_x, new_y = x + dx, y + dy
            line.set_data([x, new_x], [y, new_y])   # each link only spans its own joint-to-joint segment
            x, y = new_x, new_y

        self.end_effector_point.set_data([x], [y])

        J = self.arm._jacobian(self.current_thetas)
        self.jacobian_text.set_text(self._format_jacobian(J))

    def show(self):
        plt.show()