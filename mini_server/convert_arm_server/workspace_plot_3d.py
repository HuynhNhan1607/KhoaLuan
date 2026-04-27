"""
3D Workspace Plotter for Robot Arm

Usage:
    python workspace_plot_3d.py
    python workspace_plot_3d.py --samples 200000
    python workspace_plot_3d.py --samples 120000 --no-gripper
    python workspace_plot_3d.py --strict --show-raw
    python workspace_plot_3d.py --strict --base-front 70 --base-back 120 --base-half-width 60 --base-height 170
    python workspace_plot_3d.py --strict --base-height 170 --allow-negative-r
"""

import argparse

import matplotlib.pyplot as plt
import numpy as np

from config import d1, a2, a3, d5, SERVO_MAPPING_CONFIG, load_joint_limits


def map_servo_to_math(joint_key, servo_angle):
    cfg = SERVO_MAPPING_CONFIG[joint_key]
    return (servo_angle - cfg["offset"]) / cfg["dir"]


def _sample_joint_angles(rng, n_samples):
    limits = load_joint_limits()

    j0 = rng.uniform(limits["j0"]["min"], limits["j0"]["max"], n_samples)
    j1 = rng.uniform(limits["j1"]["min"], limits["j1"]["max"], n_samples)
    j2 = rng.uniform(limits["j2"]["min"], limits["j2"]["max"], n_samples)
    j3 = rng.uniform(limits["j3"]["min"], limits["j3"]["max"], n_samples)

    return j0, j1, j2, j3


def sample_workspace_points(n_samples=120000, seed=42, use_gripper=True, base_height=170.0):
    rng = np.random.default_rng(seed)
    j0, j1, j2, j3 = _sample_joint_angles(rng, n_samples)

    tool_len = d5 if use_gripper else 0.0

    theta0 = np.radians(map_servo_to_math("j0", j0))
    m1 = map_servo_to_math("j1", j1)
    m2 = map_servo_to_math("j2", j2)
    m3 = map_servo_to_math("j3", j3)

    t1 = np.radians(m1)
    t2 = np.radians(m2)
    t3 = np.radians(m3)

    angle_1 = t1 + np.pi / 2.0
    p1_r = a2 * np.cos(angle_1)
    p1_z = d1 + a2 * np.sin(angle_1)

    angle_2 = angle_1 - t2
    p2_r = p1_r + a3 * np.cos(angle_2)
    p2_z = p1_z + a3 * np.sin(angle_2)

    angle_3 = angle_2 - t3
    tcp_r = p2_r + tool_len * np.cos(angle_3)
    tcp_z = p2_z + tool_len * np.sin(angle_3)

    x = tcp_r * np.cos(theta0)
    y = tcp_r * np.sin(theta0)
    # Robot is mounted on top of the base box, so all arm Z points are shifted up.
    z = tcp_z + base_height

    return {
        "x": x,
        "y": y,
        "z": z,
        "tcp_r": tcp_r,
    }


def get_base_box_extents(base_front, base_back, base_half_width, base_height):
    return {
        "x_min": -base_back,
        "x_max": base_front,
        "y_min": -base_half_width,
        "y_max": base_half_width,
        "z_min": 0.0,
        "z_max": base_height,
    }


def _outside_base_box_mask(points, base_extents):
    x = points["x"]
    y = points["y"]
    z = points["z"]

    inside_box = (
        (x >= base_extents["x_min"])
        & (x <= base_extents["x_max"])
        & (y >= base_extents["y_min"])
        & (y <= base_extents["y_max"])
        & (z >= base_extents["z_min"])
        & (z <= base_extents["z_max"])
    )
    return ~inside_box


def build_strict_mask(
    points,
    min_z=0.0,
    base_clearance=35.0,
    allow_negative_r=False,
    use_base_box=True,
    base_front=70.0,
    base_back=120.0,
    base_half_width=60.0,
    base_height=170.0,
):
    x = points["x"]
    y = points["y"]
    z = points["z"]
    tcp_r = points["tcp_r"]

    checks = {
        "ground_z_ge_min": z >= min_z,
        "base_clearance": np.sqrt(x**2 + y**2) >= base_clearance,
    }

    if not allow_negative_r:
        checks["forward_radial_r_ge_0"] = tcp_r >= 0.0

    base_extents = None
    if use_base_box:
        base_extents = get_base_box_extents(base_front, base_back, base_half_width, base_height)
        checks["outside_base_box"] = _outside_base_box_mask(points, base_extents)

    mask = np.ones_like(x, dtype=bool)
    for cond in checks.values():
        mask &= cond

    return mask, checks, base_extents


def print_constraint_report(total_points, final_mask, checks):
    kept = int(np.count_nonzero(final_mask))
    print("[WORKSPACE] Constraint report")
    print(f"  Total samples: {total_points:,}")
    for name, cond in checks.items():
        pass_count = int(np.count_nonzero(cond))
        print(f"  {name}: {pass_count:,} ({100.0 * pass_count / total_points:.1f}%)")
    print(f"  Final kept: {kept:,} ({100.0 * kept / total_points:.1f}%)")


def draw_base_box(ax, base_extents):
    x_min, x_max = base_extents["x_min"], base_extents["x_max"]
    y_min, y_max = base_extents["y_min"], base_extents["y_max"]
    z_min, z_max = base_extents["z_min"], base_extents["z_max"]

    corners = np.array(
        [
            [x_min, y_min, z_min],
            [x_max, y_min, z_min],
            [x_max, y_max, z_min],
            [x_min, y_max, z_min],
            [x_min, y_min, z_max],
            [x_max, y_min, z_max],
            [x_max, y_max, z_max],
            [x_min, y_max, z_max],
        ]
    )
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]

    for start, end in edges:
        p0 = corners[start]
        p1 = corners[end]
        ax.plot(
            [p0[0], p1[0]],
            [p0[1], p1[1]],
            [p0[2], p1[2]],
            color="black",
            linewidth=1.2,
            alpha=0.7,
        )


def set_axes_equal(ax):
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    plot_radius = 0.5 * max(x_range, y_range, z_range)

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def plot_workspace_3d(
    n_samples=120000,
    seed=42,
    use_gripper=True,
    strict=False,
    min_z=0.0,
    base_clearance=35.0,
    show_raw=False,
    allow_negative_r=False,
    use_base_box=True,
    show_base_box=True,
    base_front=70.0,
    base_back=120.0,
    base_half_width=60.0,
    base_height=170.0,
):
    points = sample_workspace_points(
        n_samples=n_samples,
        seed=seed,
        use_gripper=use_gripper,
        base_height=base_height,
    )
    x = points["x"]
    y = points["y"]
    z = points["z"]

    total_samples = x.size
    title_suffix = ""

    if strict:
        strict_mask, checks, base_extents = build_strict_mask(
            points,
            min_z=min_z,
            base_clearance=base_clearance,
            allow_negative_r=allow_negative_r,
            use_base_box=use_base_box,
            base_front=base_front,
            base_back=base_back,
            base_half_width=base_half_width,
            base_height=base_height,
        )
        print_constraint_report(total_samples, strict_mask, checks)

        if not np.any(strict_mask):
            raise RuntimeError("No points left after strict filtering. Try lowering constraints.")

        if show_raw:
            x_raw, y_raw, z_raw = x, y, z

        x = x[strict_mask]
        y = y[strict_mask]
        z = z[strict_mask]
        title_suffix = " | strict"
    else:
        base_extents = None

    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection="3d")

    if strict and show_raw:
        ax.scatter(x_raw, y_raw, z_raw, c="lightgray", s=0.7, alpha=0.06, linewidths=0, label="Raw FK")

    scatter = ax.scatter(x, y, z, c=z, cmap="viridis", s=1.5, alpha=0.18, linewidths=0)
    fig.colorbar(scatter, ax=ax, shrink=0.7, pad=0.08, label="Z (mm)")

    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")

    gripper_state = "ON" if use_gripper else "OFF"
    ax.set_title(
        f"Robot Reachable Workspace (3D) | samples={n_samples:,} | gripper={gripper_state} | j0_z={base_height:.0f}mm{title_suffix}"
    )

    ax.scatter([0], [0], [base_height], c="red", s=45, label="J0")
    if strict and use_base_box and show_base_box and base_extents is not None:
        draw_base_box(ax, base_extents)
        ax.plot([], [], [], color="black", linewidth=1.2, label="Base Box")
    ax.legend(loc="upper left")

    set_axes_equal(ax)
    plt.tight_layout()
    plt.show()


def parse_args():
    parser = argparse.ArgumentParser(description="Plot 3D reachable workspace of the robot arm.")
    parser.add_argument("--samples", type=int, default=120000, help="Number of random joint samples")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    parser.add_argument(
        "--no-gripper",
        action="store_true",
        help="Ignore d5 tool length and plot wrist workspace",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Apply practical constraints: Z>=min_z, optional R>=0, base clearance, base-box collision",
    )
    parser.add_argument(
        "--min-z",
        type=float,
        default=0.0,
        help="Minimum allowed Z in strict mode",
    )
    parser.add_argument(
        "--base-clearance",
        type=float,
        default=35.0,
        help="Minimum radial distance from base axis in strict mode (mm)",
    )
    parser.add_argument(
        "--show-raw",
        action="store_true",
        help="When strict mode is on, also show raw FK cloud in gray",
    )
    parser.add_argument(
        "--allow-negative-r",
        action="store_true",
        help="Do not enforce tcp_r >= 0 (allows behind-base reach including possible Y<0)",
    )
    parser.add_argument(
        "--no-base-box",
        action="store_true",
        help="Disable base box constraint",
    )
    parser.add_argument(
        "--hide-base-box",
        action="store_true",
        help="Hide base box drawing but keep base-box constraint",
    )
    parser.add_argument(
        "--base-front",
        type=float,
        default=70.0,
        help="Base size from J0 to front (+X), in mm",
    )
    parser.add_argument(
        "--base-back",
        type=float,
        default=120.0,
        help="Base size from J0 to back (-X), in mm",
    )
    parser.add_argument(
        "--base-half-width",
        type=float,
        default=60.0,
        help="Half width of base in Y direction (both left/right), in mm",
    )
    parser.add_argument(
        "--base-height",
        type=float,
        default=170.0,
        help="Base height from ground, in mm",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    plot_workspace_3d(
        n_samples=max(1000, args.samples),
        seed=args.seed,
        use_gripper=not args.no_gripper,
        strict=args.strict,
        min_z=args.min_z,
        base_clearance=max(0.0, args.base_clearance),
        show_raw=args.show_raw,
        allow_negative_r=args.allow_negative_r,
        use_base_box=not args.no_base_box,
        show_base_box=not args.hide_base_box,
        base_front=max(0.0, args.base_front),
        base_back=max(0.0, args.base_back),
        base_half_width=max(0.0, args.base_half_width),
        base_height=max(0.0, args.base_height),
    )


if __name__ == "__main__":
    main()
