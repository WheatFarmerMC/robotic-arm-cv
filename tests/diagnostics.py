import csv
import time
import numpy as np
import matplotlib.pyplot as plt
from ..src.kinematics import Arm2D


def generate_reachable_targets(arm, n_points, seed=43):
    """
    Sample valid joint angles (within arm.theta_min/theta_max) and run FK
    to get target points. Every point is guaranteed reachable, since it's
    exactly where a legally-configured arm can be — unlike sampling by
    distance/angle, which assumes full 360° freedom at joint 1 and ignores
    joint limits entirely.
    """
    rng = np.random.default_rng(seed)
    sampled_thetas = rng.uniform(arm.theta_min, arm.theta_max, size=(n_points, arm.n_links))

    targets = []
    for thetas in sampled_thetas:
        vectors = arm.forward_kinematics(*thetas)
        x = sum(v[0] for v in vectors)
        y = sum(v[1] for v in vectors)
        targets.append((x, y))
    return targets

def safe_default_pose(arm):
    theta1 = (arm.theta_min[0] + arm.theta_max[0]) / 2
    theta2 = (arm.theta_min[1] + arm.theta_max[1]) / 2
    if abs(theta2) < 1e-3:          # midpoint landed on the singularity — nudge off it
        theta2 += 0.3
    return (theta1, theta2)

def generate_click_path(arm, n_points, step_size_deg=15, seed=43):
    """
    Simulates a REALISTIC sequence of clicks: each target is a modest random
    walk step (in joint-angle space) from the last one, rather than an
    independent random jump anywhere in the workspace. This matches how
    simulation.py is actually used — a person clicks somewhere near where
    the arm already is, not at a uniformly random point every time.
    """
    rng = np.random.default_rng(seed)
    thetas = safe_default_pose(arm)
    targets = []
    for _ in range(n_points):
        step = np.radians(rng.uniform(-step_size_deg, step_size_deg, size=arm.n_links))
        thetas = np.clip(thetas + step, arm.theta_min, arm.theta_max)
        vectors = arm.forward_kinematics(*thetas)
        x = sum(v[0] for v in vectors)
        y = sum(v[1] for v in vectors)
        targets.append((x, y))
    return targets

def benchmark_jacobian_warmstart(arm, targets):
    """
    The realistic-usage number: each solve warm-starts from wherever the
    PREVIOUS solve ended, exactly how simulation.py behaves as someone
    clicks around. Pair this with generate_click_path (NOT
    generate_reachable_targets, which produces independent random jumps)
    for a fair comparison.
    """
    times, errors, iterations = [], [], []
    current_pose = safe_default_pose(arm)
    for x, y in targets:
        start = time.perf_counter()
        thetas, diag = arm.inverse_kinematics_jacobian_pinv(
            x, y, theta_init=current_pose, return_diagnostics=True
        )
        times.append(time.perf_counter() - start)
        errors.append(position_error(arm, thetas, (x, y)))
        iterations.append(diag["iterations"])
        current_pose = np.array(thetas)
    return np.array(times), np.array(errors), np.array(iterations)





def position_error(arm, thetas, target):
    vectors = arm.forward_kinematics(*thetas)
    x = sum(v[0] for v in vectors)
    y = sum(v[1] for v in vectors)
    return np.hypot(x - target[0], y - target[1])


def benchmark_geometric(arm, targets):
    times, errors = [], []
    for x, y in targets:
        start = time.perf_counter()
        thetas = arm.inverse_kinematics_geometric(x, y)
        times.append(time.perf_counter() - start)
        errors.append(position_error(arm, thetas, (x, y)))
    return np.array(times), np.array(errors)

def diagnose_failures(arm, targets, theta_init=None, n_show=5):
    print("\n--- Failed/maxed-out trials ---")
    shown = 0
    init = theta_init if theta_init is not None else safe_default_pose(arm)
    for x, y in targets:
        thetas, diag = arm.inverse_kinematics_jacobian_pinv(
            
            x, y, theta_init=init, return_diagnostics=True
        )
        if diag["iterations"] >= 149:  # hit the cap
            at_limit = np.isclose(thetas, arm.theta_min, atol=1e-3) | np.isclose(thetas, arm.theta_max, atol=1e-3)
            print(f"target=({x:.2f},{y:.2f})  final_thetas={np.round(thetas,3)}  "
                  f"error={diag['final_error']:.3f}  any_joint_at_limit={at_limit.any()}")
            shown += 1
            if shown >= n_show:
                break

def benchmark_jacobian(arm, targets, theta_init=None):
    times, errors, iterations = [], [], []
    for x, y in targets:
        init = theta_init if theta_init is not None else safe_default_pose(arm)
        start = time.perf_counter()
        thetas, diag = arm.inverse_kinematics_jacobian_pinv(
            x, y, theta_init=init, return_diagnostics=True
        )
        times.append(time.perf_counter() - start)
        errors.append(position_error(arm, thetas, (x, y)))
        iterations.append(diag["iterations"])
        #if diag["iterations"] == 150:
            #print(x, y)
    return np.array(times), np.array(errors), np.array(iterations)

def summarize(name, times, errors, iterations=None):
    print(f"\n{name}")
    print(f"  mean time:   {times.mean()*1e6:8.2f} µs  (median {np.median(times)*1e6:.2f} µs)")
    print(f"  mean error:  {errors.mean():.6f}")
    if iterations is not None:
        print(f"  mean iters:  {iterations.mean():.1f}  (max {iterations.max()})")


def save_csv(path, geo_times, geo_errors, jac_times, jac_errors, jac_iterations):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["trial", "geometric_time_s", "geometric_error",
                          "jacobian_time_s", "jacobian_error", "jacobian_iterations"])
        for i in range(len(geo_times)):
            writer.writerow([i, geo_times[i], geo_errors[i],
                              jac_times[i], jac_errors[i], jac_iterations[i]])


def plot_comparison(geo_times, jac_times, jac_iterations):
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))

    axes[0].bar(["Geometric", "Jacobian (pinv)"],
                [geo_times.mean() * 1e6, jac_times.mean() * 1e6],
                color=["r", "b"])
    axes[0].set_ylabel("Mean time per call (µs)")
    axes[0].set_title("Execution time")

    axes[1].hist(jac_iterations, bins=range(1, int(jac_iterations.max()) + 2),
                 color="b", edgecolor="black")
    axes[1].set_xlabel("Iterations to converge")
    axes[1].set_ylabel("Count")
    axes[1].set_title("Jacobian convergence")

    fig.tight_layout()
    plt.show()


def main():
    arm = Arm2D(link1_length=5.0, link2_length=3.0)
    n_trials = 500
    click_targets = generate_click_path(arm, n_trials)
    # Scenario A: independent random targets (worst-case stress test)
    random_targets = generate_reachable_targets(arm, n_trials)
    geo_times, geo_errors = benchmark_geometric(arm, random_targets)
    jac_times, jac_errors, jac_iterations = benchmark_jacobian_warmstart(arm, click_targets)

    # Scenario B: click-path targets (realistic usage — matches simulation.py)
    
    warm_times, warm_errors, warm_iterations = benchmark_jacobian_warmstart(arm, click_targets)

    summarize("Geometric (closed-form)", geo_times, geo_errors)
    summarize("Jacobian, independent random targets", jac_times, jac_errors, jac_iterations)
    summarize("Jacobian, realistic click-path (warm-started)", warm_times, warm_errors, warm_iterations)

    save_csv("ik_benchmark_results.csv", geo_times, geo_errors, jac_times, jac_errors, jac_iterations)
    print("\nRaw per-trial data written to ik_benchmark_results.csv")

    plot_comparison(geo_times, jac_times, jac_iterations)


if __name__ == "__main__":
    main()