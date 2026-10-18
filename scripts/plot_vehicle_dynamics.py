"""
Vehicle Dynamics & Suspension Free Body Diagram Generator
Generates Figure 1: Vehicle Free Body Diagram & Suspension Load Distribution
Outputs saved to scripts/vehicle_dynamics_fbd.png
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import os

def generate_vehicle_fbd():
    scripts_dir = os.path.dirname(os.path.abspath(__file__))
    project_dir = os.path.dirname(scripts_dir)
    results_dir = os.path.join(project_dir, 'results')
    os.makedirs(results_dir, exist_ok=True)
    out_path = os.path.join(results_dir, 'vehicle_dynamics_fbd.png')

    fig, ax = plt.subplots(figsize=(13, 7.2), dpi=300)
    ax.set_facecolor('#f8fafc')
    fig.patch.set_facecolor('#ffffff')

    # Ground line
    ax.plot([-0.6, 4.8], [0, 0], color='#334155', lw=3, zorder=2)
    for x in np.arange(-0.5, 4.7, 0.18):
        ax.plot([x, x - 0.10], [0, -0.10], color='#94a3b8', lw=1.5)

    # Dimensions
    x_front = 0.85
    x_rear = 3.935  # wheelbase = 3.085 m
    r_wheel = 0.42

    # Draw Wheels
    wheel_f = plt.Circle((x_front, r_wheel), r_wheel, facecolor='#1e293b', edgecolor='#0f172a', lw=3, zorder=3)
    rim_f = plt.Circle((x_front, r_wheel), r_wheel * 0.52, facecolor='#94a3b8', edgecolor='#334155', lw=2, zorder=4)
    hub_f = plt.Circle((x_front, r_wheel), r_wheel * 0.18, facecolor='#e2e8f0', edgecolor='#0f172a', lw=1.5, zorder=5)

    wheel_r = plt.Circle((x_rear, r_wheel), r_wheel, facecolor='#1e293b', edgecolor='#0f172a', lw=3, zorder=3)
    rim_r = plt.Circle((x_rear, r_wheel), r_wheel * 0.52, facecolor='#94a3b8', edgecolor='#334155', lw=2, zorder=4)
    hub_r = plt.Circle((x_rear, r_wheel), r_wheel * 0.18, facecolor='#e2e8f0', edgecolor='#0f172a', lw=1.5, zorder=5)

    ax.add_patch(wheel_f)
    ax.add_patch(rim_f)
    ax.add_patch(hub_f)
    ax.add_patch(wheel_r)
    ax.add_patch(rim_r)
    ax.add_patch(hub_r)

    # Vehicle chassis/body outline
    body_pts = [
        [0.10, r_wheel * 0.85],
        [0.10, 1.15],
        [0.75, 1.18],
        [1.40, 1.75],
        [2.55, 1.75],
        [2.60, 1.22],
        [4.55, 1.22],
        [4.55, r_wheel * 0.85],
        [3.45, r_wheel * 0.85],
        [3.45, r_wheel * 1.35],
        [1.35, r_wheel * 1.35],
        [1.35, r_wheel * 0.85],
        [0.10, r_wheel * 0.85]
    ]
    body_poly = patches.Polygon(body_pts, closed=True, facecolor='#e2e8f0', edgecolor='#334155', lw=2.5, zorder=2)
    ax.add_patch(body_poly)

    # Cabin glass
    cabin_pts = [[1.45, 1.26], [1.54, 1.68], [2.48, 1.68], [2.52, 1.26]]
    cabin_poly = patches.Polygon(cabin_pts, closed=True, facecolor='#bfdbfe', edgecolor='#3b82f6', alpha=0.75, lw=1.8, zorder=3)
    ax.add_patch(cabin_poly)

    # Leaf Spring indication at rear axle
    ls_x = np.linspace(x_rear - 0.65, x_rear + 0.65, 60)
    ls_y = r_wheel * 0.65 - 0.09 * (1 - ((ls_x - x_rear) / 0.65)**2)
    ax.plot(ls_x, ls_y, color='#b91c1c', lw=4.5, zorder=6, label='Leaf Spring Pack')
    ax.scatter([x_rear - 0.65, x_rear + 0.65], [r_wheel * 0.65, r_wheel * 0.65], color='#b91c1c', s=60, zorder=7)

    # Center of Gravity (CG)
    x_cg = x_front + 1.851
    y_cg = 0.750
    cg_circle = plt.Circle((x_cg, y_cg), 0.10, facecolor='#f59e0b', edgecolor='#92400e', lw=2.5, zorder=7)
    ax.add_patch(cg_circle)
    wedge1 = patches.Wedge((x_cg, y_cg), 0.10, 0, 90, facecolor='#92400e', zorder=8)
    wedge2 = patches.Wedge((x_cg, y_cg), 0.10, 180, 270, facecolor='#92400e', zorder=8)
    ax.add_patch(wedge1)
    ax.add_patch(wedge2)

    # CG Label Box
    ax.annotate('CG (Center of Gravity)\nh_cg = 0.750 m', xy=(x_cg, y_cg), xytext=(x_cg - 0.95, y_cg + 0.65),
                fontsize=11, fontweight='bold', color='#78350f',
                arrowprops=dict(arrowstyle='->', color='#b45309', lw=1.8, connectionstyle='arc3,rad=-0.15'),
                bbox=dict(boxstyle='round,pad=0.5', facecolor='#fef3c7', edgecolor='#f59e0b', lw=1.5), zorder=9)

    # Force Vectors
    # Weight vector at CG
    ax.annotate('', xy=(x_cg, y_cg - 0.58), xytext=(x_cg, y_cg),
                arrowprops=dict(facecolor='#d97706', edgecolor='#92400e', width=3.5, headwidth=11), zorder=8)
    ax.text(x_cg + 0.12, y_cg - 0.40, 'Total Weight W = M · g\nW = 27,468.0 N\n(GVW = 2,800 kg)', 
            fontsize=11, fontweight='bold', color='#92400e',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='#fffbeb', edgecolor='#fcd34d', lw=1.2))

    # Front axle reaction
    ax.annotate('', xy=(x_front, 1.45), xytext=(x_front, 0),
                arrowprops=dict(facecolor='#2563eb', edgecolor='#1d4ed8', width=3.5, headwidth=11), zorder=8)
    ax.text(x_front - 0.78, 1.55, 'Front Axle Reaction R_f\nR_f = 10,987.2 N\n(40% Static Load)', 
            fontsize=10.5, fontweight='bold', color='#1e40af',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='#eff6ff', edgecolor='#93c5fd', lw=1.2))

    # Rear axle reaction
    ax.annotate('', xy=(x_rear, 1.55), xytext=(x_rear, 0),
                arrowprops=dict(facecolor='#16a34a', edgecolor='#15803d', width=4, headwidth=12), zorder=8)
    ax.text(x_rear - 0.65, 1.65, 'Rear Axle Reaction R_r = 16,480.8 N (60% Total)\n↳ Static per spring: F_v,static = 8,240.4 N\n↳ 2.0G Dynamic bump: F_v,bump = 16,480.8 N', 
            fontsize=10.5, fontweight='bold', color='#14532d',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='#f0fdf4', edgecolor='#86efac', lw=1.5))

    # Dimension Lines
    y_dim1 = -0.38
    ax.annotate('', xy=(x_rear, y_dim1), xytext=(x_front, y_dim1),
                arrowprops=dict(arrowstyle='<->', color='#1e293b', lw=1.8))
    ax.plot([x_front, x_front], [-0.05, y_dim1 - 0.10], color='#64748b', lw=1.2, ls='--')
    ax.plot([x_rear, x_rear], [-0.05, y_dim1 - 0.10], color='#64748b', lw=1.2, ls='--')
    ax.text((x_front + x_rear) / 2, y_dim1 + 0.08, 'Wheelbase L_wb = 3.085 m', ha='center', va='bottom', fontsize=11.5, fontweight='bold', color='#0f172a',
            bbox=dict(boxstyle='round,pad=0.2', facecolor='#ffffff', edgecolor='#cbd5e1', lw=1))

    # Sub-dimensions a and b
    y_dim2 = -0.72
    ax.annotate('', xy=(x_cg, y_dim2), xytext=(x_front, y_dim2), arrowprops=dict(arrowstyle='<->', color='#475569', lw=1.5))
    ax.annotate('', xy=(x_rear, y_dim2), xytext=(x_cg, y_dim2), arrowprops=dict(arrowstyle='<->', color='#475569', lw=1.5))
    ax.plot([x_cg, x_cg], [y_cg - 0.15, y_dim2 - 0.10], color='#d97706', lw=1.2, ls=':')
    ax.plot([x_front, x_front], [y_dim1, y_dim2 - 0.10], color='#64748b', lw=1.2, ls='--')
    ax.plot([x_rear, x_rear], [y_dim1, y_dim2 - 0.10], color='#64748b', lw=1.2, ls='--')
    ax.text((x_front + x_cg) / 2, y_dim2 - 0.08, 'Front to CG: a = 1.851 m', ha='center', va='top', fontsize=10, fontweight='bold', color='#334155')
    ax.text((x_cg + x_rear) / 2, y_dim2 - 0.08, 'CG to Rear: b = 1.234 m', ha='center', va='top', fontsize=10, fontweight='bold', color='#334155')

    # Header
    ax.text(0.10, 2.40, 'Vehicle Free Body Diagram & Suspension Load Distribution', fontsize=16, fontweight='bold', color='#0f172a')
    ax.text(0.10, 2.22, 'Light Commercial Vehicle (1-Tonne Laden Pickup Truck) | Total Gravitational Mass M = 2,800 kg', fontsize=11.5, color='#475569')

    ax.set_xlim(-0.7, 4.9)
    ax.set_ylim(-1.0, 2.55)
    ax.axis('off')
    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f'[OK] Generated: {out_path}')

if __name__ == '__main__':
    generate_vehicle_fbd()
