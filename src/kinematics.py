
from src.config import THETA1_MIN, THETA1_MAX, THETA2_MIN, THETA2_MAX, THETA3_MIN, THETA3_MAX, ELBOW_CONFIG
import numpy as np
import math

def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


class Arm2D:
    def __init__(self, link1_length, link2_length, link3_length=None):
        self.link1_length = link1_length
        self.link2_length = link2_length
        self.link3_length = link3_length

        lengths = [link1_length, link2_length]
        if link3_length is not None:
            lengths.append(link3_length)
        self.link_lengths = np.array(lengths, dtype=float)   # everything Jacobian-related is driven off this
        self.n_links = len(self.link_lengths)

        bounds = [(THETA1_MIN, THETA1_MAX), (THETA2_MIN, THETA2_MAX), (THETA3_MIN, THETA3_MAX)][:self.n_links]
        self.theta_min = np.array([b[0] for b in bounds])
        self.theta_max = np.array([b[1] for b in bounds])


    # Forward kinematics

    def _joint_positions(self, thetas):
        """
        Vectorized FK core. `thetas` = [theta1, theta2, ...].
        Returns shape (n_links+1, 2): row 0 = base (origin), row i = position
        after link i, row -1 = end effector. Every other method (forward
        kinematics, error calc, Jacobian) reuses this single computation
        instead of re-deriving joint positions from scratch.
        """
        cumulative_angles = np.cumsum(thetas)
        segments = np.column_stack((
            self.link_lengths * np.cos(cumulative_angles),
            self.link_lengths * np.sin(cumulative_angles),
        ))
        joints = np.empty((self.n_links + 1, 2))
        joints[0] = 0.0
        np.cumsum(segments, axis=0, out=joints[1:])
        return joints

    def forward_kinematics(self, *thetas, center=(0, 0)):
        """Same interface as before: forward_kinematics(theta1, theta2[, theta3]) -> (vector1, vector2[, vector3])."""
        thetas = np.array(thetas, dtype=float)
        assert len(thetas) == self.n_links, f"Expected {self.n_links} angles, got {len(thetas)}"
        joints = self._joint_positions(thetas)
        vectors = joints[1:] - joints[:-1]      # each link's own (dx, dy), not cumulative
        return tuple(tuple(v) for v in vectors)

    # Geometric IK — unchanged, 2-link only

    def inverse_kinematics_geometric(self, x, y):
        if self.n_links != 2:
            raise NotImplementedError(
                "Closed-form geometric IK only supports 2 links. "
                "Use inverse_kinematics_jacobian_pinv for a 3-link arm."
            )

        distance = math.sqrt(x**2 + y**2)
        if distance > (self.link1_length + self.link2_length):
            raise ValueError("Target position is out of reach.")
        if distance < abs(self.link1_length - self.link2_length):
            raise ValueError("Target position is too close to reach.")

        cos_theta2 = clamp((distance**2 - self.link1_length**2 - self.link2_length**2) /
                            (2 * self.link1_length * self.link2_length), -1.0, 1.0)
        theta2_mag = math.acos(cos_theta2)

        candidates = []
        for sign in (1, -1):
            theta2 = sign * theta2_mag
            theta1 = math.atan2(y, x) - math.atan2(
                self.link2_length * math.sin(theta2),
                self.link1_length + self.link2_length * math.cos(theta2)
            )
            in_bounds = (self.theta_min[0] <= theta1 <= self.theta_max[0] and
                        self.theta_min[1] <= theta2 <= self.theta_max[1])
            candidates.append((theta1, theta2, in_bounds))

        for theta1, theta2, in_bounds in candidates:
            if in_bounds:
                return (theta1, theta2)

        # Neither branch fits cleanly — fall back to whichever clamps closest
        best, best_error = None, float("inf")
        for theta1, theta2, _ in candidates:
            c1 = clamp(theta1, self.theta_min[0], self.theta_max[0])
            c2 = clamp(theta2, self.theta_min[1], self.theta_max[1])
            ex = ey = 0.0
            for vx, vy in self.forward_kinematics(c1, c2):
                ex += vx; ey += vy
            err = math.hypot(x - ex, y - ey)
            if err < best_error:
                best_error, best = err, (c1, c2)
        return best


    # Jacobian — works for 2 or 3 (or N) links
    def _jacobian(self, thetas, joints=None):
        """
        Analytic Jacobian, shape (2, n_links). Column i = how the end
        effector moves per radian of joint i. Pass `joints` if you've
        already computed it this iteration, to skip a redundant FK pass.
        """
        if joints is None:
            joints = self._joint_positions(thetas)

        end_effector = joints[-1]
        lever_arms = end_effector - joints[:-1]           # (n_links, 2) — vectorized, no per-joint python loopgdg
        J = np.column_stack((-lever_arms[:, 1], lever_arms[:, 0]))
        return J.T                                          # (2, n_links)

    def inverse_kinematics_jacobian_pinv(self, x, y, theta_init, max_iterations=150,
                                        tolerance=1e-4, damping=0.1, return_diagnostics=False, max_step=0.5):
        distance = math.sqrt(x**2 + y**2)
        if distance > (self.link1_length + self.link2_length + (self.link3_length if self.link3_length is not None else 0)):
            raise ValueError("Target position is out of reach.")
        if distance < abs(self.link1_length - self.link2_length - (self.link3_length if self.link3_length is not None else 0)):
            raise ValueError("Target position is too close to reach.")
        thetas = np.array(theta_init, dtype=float)
        target = np.array([x, y])
        damping_matrix = (damping ** 2) * np.eye(2)

        iterations_used = max_iterations
        
        for i in range(max_iterations):
            for theta in thetas:
                if theta == 0:
                    theta = 1e-3  # Avoid singularity at zero angles
            joints = self._joint_positions(thetas)
            error = target - joints[-1]

            if np.linalg.norm(error) < tolerance:
                iterations_used = i
                break

            J = self._jacobian(thetas, joints=joints)

            # Unconstrained step, to see which direction each joint WANTS to move
            J_pinv = J.T @ np.linalg.inv(J @ J.T + damping_matrix)
            delta_theta = J_pinv @ error
            step_norm = np.linalg.norm(delta_theta)
            if step_norm > max_step:
                delta_theta *= max_step / step_norm
            # A joint is "saturated" if it's sitting at a limit AND the proposed
            # step would push it further past that limit
            at_lower = (thetas <= self.theta_min) & (delta_theta < 0)
            at_upper = (thetas >= self.theta_max) & (delta_theta > 0)
            freeze_mask = at_lower | at_upper

            if np.any(freeze_mask):
                # Zero out frozen joints' columns — solver redistributes the
                # correction across whichever joints are still free to move.
                # (If EVERY joint happens to be frozen, J_active is all zero, but
                # the damping term keeps the inverse well-defined and correctly
                # yields delta_theta = 0 — no movement possible, which is correct.)
                J_active = J.copy()
                J_active[:, freeze_mask] = 0.0
                J_pinv = J_active.T @ np.linalg.inv(J_active @ J_active.T + damping_matrix)
                delta_theta = J_pinv @ error
                delta_theta[freeze_mask] = 0.0

            thetas += delta_theta
            np.clip(thetas, self.theta_min, self.theta_max, out=thetas)  # safety net for tiny float drift only

            iterations_used = i + 1

        if return_diagnostics:
            final_error = np.linalg.norm(target - self._joint_positions(thetas)[-1])
            return tuple(thetas), {"iterations": iterations_used, "final_error": final_error}
        return tuple(thetas)
    def inverse_kinematics_jacobian_log(self, x, y, theta_init, max_iterations=150,
                                        tolerance=1e-4, damping=0.1, return_diagnostics=False, max_step=0.5):
        distance = math.sqrt(x**2 + y**2)
        if distance > (self.link1_length + self.link2_length + (self.link3_length if self.link3_length is not None else 0)):
            raise ValueError("Target position is out of reach.")
        if distance < abs(self.link1_length - self.link2_length - (self.link3_length if self.link3_length is not None else 0)):
            raise ValueError("Target position is too close to reach.")
        thetas = np.array(theta_init, dtype=float)
        target = np.array([x, y])
        damping_matrix = (damping ** 2) * np.eye(2)

        iterations_used = max_iterations
        logthetas = []  # To store the sequence of intermediate poses
        logjacobs = []  # To store the sequence of Jacobians
        for i in range(max_iterations):
            logthetas.append(tuple(thetas))  # Store the current pose

            joints = self._joint_positions(thetas)
            error = target - joints[-1]

            if np.linalg.norm(error) < tolerance:
                iterations_used = i
                break

            J = self._jacobian(thetas, joints=joints)
            logjacobs.append(J.copy())  # Store the current Jacobian

            # Unconstrained step, to see which direction each joint WANTS to move
            J_pinv = J.T @ np.linalg.inv(J @ J.T + damping_matrix)
            delta_theta = J_pinv @ error

            # A joint is "saturated" if it's sitting at a limit AND the proposed
            # step would push it further past that limit
            at_lower = (thetas <= self.theta_min) & (delta_theta < 0)
            at_upper = (thetas >= self.theta_max) & (delta_theta > 0)
            freeze_mask = at_lower | at_upper

            if np.any(freeze_mask):
                # Zero out frozen joints' columns — solver redistributes the
                # correction across whichever joints are still free to move.
                # (If EVERY joint happens to be frozen, J_active is all zero, but
                # the damping term keeps the inverse well-defined and correctly
                # yields delta_theta = 0 — no movement possible, which is correct.)
                J_active = J.copy()
                J_active[:, freeze_mask] = 0.0
                J_pinv = J_active.T @ np.linalg.inv(J_active @ J_active.T + damping_matrix)
                delta_theta = J_pinv @ error
                delta_theta[freeze_mask] = 0.0

            print(delta_theta)
            thetas += delta_theta
            np.clip(thetas, self.theta_min, self.theta_max, out=thetas)  # safety net for tiny float drift only

            iterations_used = i + 1

        if return_diagnostics:
            final_error = np.linalg.norm(target - self._joint_positions(thetas)[-1])
            return tuple(thetas), {"iterations": iterations_used, "final_error": final_error}
        return tuple(thetas), logthetas, logjacobs  # Return the sequence of intermediate poses and Jacobians