#!/usr/bin/env python3
"""
plotting_script_v2.py

Plots CSVs produced by data_agent_v3.py. Auto-detects which solver columns are
present and adjusts the layout accordingly.

Unit conversion (all plotted velocities are in m/s for consistent comparison
against ball velocity):
    - Linear rod:    v_y [m/s] = omega_d_y [rad/s] * RPP
                     where RPP = (field_height / steps_across_field) * (steps_per_rev / (2*pi))
                     (this inverts the scaling in velocity_ctrl.py so ball and rod velocities
                      share one y-axis cleanly).
    - Angular rod:   v_tangential [m/s] = omega_d_x [rad/s] * L_P
                     where L_P is the player leg length.

Use --raw_rads to plot omega commands in rad/s on their own subplot instead.

Example:
    python3 plotting_script_v2.py --input y_test.csv --output y_plot.png
    python3 plotting_script_v2.py --input x_test.csv --output x_plot.png --raw_rads
    python3 plotting_script_v2.py --input xy_test.csv --output xy_plot.png
"""

import argparse
import math
import pandas as pd
import matplotlib.pyplot as plt


# ============================================================
# Physical / calibration constants (defaults match the
# config used in velocity_ctrl.py / foosball system)
# ============================================================
# Linear:  (120 steps across the field / 525 encoder steps across) and (2*pi / 400 steps per rev)
#          -> data.linear.x [m/s] = omega_d [rad/s] * rpp
#          -> so the inverse we want is v_y [m/s] = omega_d [rad/s] * rpp
DEFAULT_PPS = 120.0 / 525.0              # pixels-per-step (legacy name from velocity_ctrl)
DEFAULT_STEPS_PER_REV = 400.0
DEFAULT_FIELD_HEIGHT_M = 0.340           # field_height in meters (from config)
DEFAULT_STEPS_ACROSS_FIELD = 525.0       # steps_across_field

# Leg length (foot distance from rod center) used for angular -> tangential speed.
DEFAULT_LP = 0.045  # meters


def compute_rpp(field_height_m, steps_across_field, steps_per_rev):
    """
    Inverse of the conversion used by velocity_ctrl.py to go from
    rad/s (omega_d) back to m/s (linear speed of the rod).

    velocity_ctrl.py:
        pps = 120 / 525       # using the legacy pixel convention, where 120 = field height in pixels
        rpp = (1/pps) * (2*pi / steps_per_rev)

    In metric, that becomes:
        pps_m = steps_across_field / field_height_m
        rpp_m = (1/pps_m) * (2*pi / steps_per_rev)
    This is the multiplier that converts rad/s -> m/s.
    """
    pps_m = steps_across_field / field_height_m
    rpp_m = (1.0 / pps_m) * (2.0 * math.pi / steps_per_rev)
    return rpp_m


# ============================================================
# CLI
# ============================================================
parser = argparse.ArgumentParser(description="Foosball Experiment Plotter (v2)")
parser.add_argument("--input", type=str, required=True, help="Input CSV file.")
parser.add_argument("--output", type=str, default="foosball_tracking_plot.png", help="Output PNG filename.")
parser.add_argument("--raw_rads", action="store_true",
                    help="Plot omega commands in rad/s (native units) instead of converting to m/s.")
parser.add_argument("--field_height", type=float, default=DEFAULT_FIELD_HEIGHT_M,
                    help=f"Field height in meters (default: {DEFAULT_FIELD_HEIGHT_M}).")
parser.add_argument("--steps_across_field", type=float, default=DEFAULT_STEPS_ACROSS_FIELD,
                    help=f"Encoder steps across the field (default: {DEFAULT_STEPS_ACROSS_FIELD}).")
parser.add_argument("--steps_per_rev", type=float, default=DEFAULT_STEPS_PER_REV,
                    help=f"Motor steps per revolution (default: {DEFAULT_STEPS_PER_REV}).")
parser.add_argument("--LP", type=float, default=DEFAULT_LP,
                    help=f"Player leg length in meters (default: {DEFAULT_LP}).")

args = parser.parse_args()

RPP = compute_rpp(args.field_height, args.steps_across_field, args.steps_per_rev)
LP = args.LP


# ============================================================
# Load & clean
# ============================================================
df = pd.read_csv(args.input)
df = df.apply(pd.to_numeric, errors='coerce').dropna()

time = df['time'].to_numpy()

has_ball = 'ball_y' in df.columns

# Detect Y-solver (linear) rods
linear_rods = sorted({
    int(c.split('_')[0].replace('rod', ''))
    for c in df.columns
    if c.startswith('rod') and c.endswith('_y') and 'player' in c
})

# Detect X-solver (angular) rods
angular_rods = sorted({
    int(c.split('_')[0].replace('rod', ''))
    for c in df.columns
    if c.startswith('rod') and c.endswith('_theta') and 'player' in c
})

has_y_solver = len(linear_rods) > 0 and any(c.startswith('omega_d_y') for c in df.columns)
has_x_solver = len(angular_rods) > 0 and any(c.startswith('omega_d_x') for c in df.columns)

print(f"[plot] Detected linear rods: {linear_rods}")
print(f"[plot] Detected angular rods: {angular_rods}")
print(f"[plot] Y-solver data present: {has_y_solver}")
print(f"[plot] X-solver data present: {has_x_solver}")


# ============================================================
# Build figure
# ============================================================
# Subplots we'll show (order matters):
#   1. linear_positions  — ball_y + rod Y-positions     (if Y-solver present)
#   2. angular_positions — θ for angular rods           (if X-solver present)
#   3. y_velocities      — ball_vy vs rod Y velocity    (if Y-solver present)
#   4. x_velocities      — ball_vx vs foot tangential   (if X-solver present)
panels = []
if linear_rods:
    panels.append('linear_positions')
if angular_rods:
    panels.append('angular_positions')
if has_y_solver:
    panels.append('y_velocities')
if has_x_solver:
    panels.append('x_velocities')

# Height ratios: position panels taller than velocity panels
height_ratios = []
for p in panels:
    if p in ('linear_positions', 'angular_positions'):
        height_ratios.append(3)
    else:
        height_ratios.append(2)

try:
    plt.style.use('seaborn-paper')
except OSError:
    # Newer matplotlib uses a different style name
    try:
        plt.style.use('seaborn-v0_8-paper')
    except OSError:
        pass

fig, axes = plt.subplots(
    len(panels), 1,
    figsize=(10, 1.2 * sum(height_ratios)),
    sharex=True,
    gridspec_kw={'height_ratios': height_ratios}
)
if len(panels) == 1:
    axes = [axes]

ax_map = dict(zip(panels, axes))


# ------------------------------------------------------------
# Panel: linear rod positions
# ------------------------------------------------------------
if 'linear_positions' in ax_map:
    ax = ax_map['linear_positions']

    if has_ball:
        ax.plot(time, df['ball_y'].to_numpy(),
                label='Ball Y Position', linewidth=2.5, color='black')

    linestyles = ['-', '--', ':']
    for m in linear_rods:
        for i in range(1, 4):
            col = f'rod{m}_player{i}_y'
            if col in df.columns:
                ax.plot(time, df[col].to_numpy(),
                        label=f'Rod{m} Player{i}' if i == 1 else None,
                        linestyle=linestyles[(i - 1) % len(linestyles)],
                        linewidth=2)

    ax.set_ylabel('Y Position (m)')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, linestyle='--', alpha=0.5)


# ------------------------------------------------------------
# Panel: angular rod positions
# ------------------------------------------------------------
if 'angular_positions' in ax_map:
    ax = ax_map['angular_positions']

    linestyles = ['-', '--', ':']
    for m in angular_rods:
        # All 3 players on an angular rod share the same theta, so just plot player 1.
        col = f'rod{m}_player1_theta'
        if col in df.columns:
            ax.plot(time, df[col].to_numpy(),
                    label=f'Rod{m} θ',
                    linewidth=2)

    ax.set_ylabel('Angle θ (rad)')
    ax.axhline(0, color='black', linewidth=1, alpha=0.6)
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, linestyle='--', alpha=0.5)


# ------------------------------------------------------------
# Panel: Y velocities — ball_vy vs rod Y velocity
# ------------------------------------------------------------
if 'y_velocities' in ax_map:
    ax = ax_map['y_velocities']

    if has_ball:
        ax.plot(time, df['ball_vy'].to_numpy(),
                label='Ball Y Velocity', color='purple',
                linewidth=2, linestyle='--')

    for m in linear_rods:
        col = f'omega_d_y_rod{m}'
        if col not in df.columns:
            continue
        omega = df[col].to_numpy()
        if args.raw_rads:
            ax.plot(time, omega, label=f'ω_d,y rod{m} (rad/s)',
                    color='red', linewidth=2)
        else:
            ax.plot(time, omega * RPP, label=f'Rod{m} Y Velocity (m/s)',
                    color='red', linewidth=2)

    ax.set_ylabel('Y Velocity (rad/s)' if args.raw_rads else 'Y Velocity (m/s)')
    ax.axhline(0, color='black', linewidth=1, alpha=0.6)
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, linestyle='--', alpha=0.5)


# ------------------------------------------------------------
# Panel: X velocities — ball_vx vs foot tangential velocity
# ------------------------------------------------------------
if 'x_velocities' in ax_map:
    ax = ax_map['x_velocities']

    if has_ball and 'ball_vx' in df.columns:
        ax.plot(time, df['ball_vx'].to_numpy(),
                label='Ball X Velocity', color='purple',
                linewidth=2, linestyle='--')

    for m in angular_rods:
        col = f'omega_d_x_rod{m}'
        if col not in df.columns:
            continue
        omega = df[col].to_numpy()
        if args.raw_rads:
            ax.plot(time, omega, label=f'ω_d,x rod{m} (rad/s)',
                    color='darkorange', linewidth=2)
        else:
            ax.plot(time, omega * LP, label=f'Rod{m} Foot Tangential (m/s)',
                    color='darkorange', linewidth=2)

    ax.set_ylabel('X Velocity (rad/s)' if args.raw_rads else 'X Velocity (m/s)')
    ax.axhline(0, color='black', linewidth=1, alpha=0.6)
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, linestyle='--', alpha=0.5)


axes[-1].set_xlabel('Time (s)')
axes[0].set_title('Foosball Rod Tracking Performance', fontsize=14)

plt.tight_layout()
plt.savefig(args.output, dpi=600)
print(f"[plot] Saved {args.output}")
plt.show()
