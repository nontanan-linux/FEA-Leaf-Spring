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

    fig, ax = plt.subplots(figsize=(13.5, 7.5), dpi=300)
    ax.set_facecolor('#f8fafc')
    fig.patch.set_facecolor('#ffffff')

    # Dimensions & Key Coordinates
    x_front = 0.85
    x_rear = 3.935          # Wheelbase L_wb = 3.085 m
    r_wheel = 0.40          # Wheel rolling radius (m)
    y_belly = 0.44          # Flat vehicle underbody / rocker sill line (m)
    x_cg = x_front + 1.851  # CG longitudinal position: 2.701 m (a = 1.851 m, b = 1.234 m)
    y_cg = 0.750            # CG height above ground h_cg = 0.750 m

    # Ground line & Hatching
    ax.plot([-0.75, 4.85], [0, 0], color='#334155', lw=3, zorder=2)
    for x in np.arange(-0.70, 4.80, 0.16):
        ax.plot([x, x - 0.10], [0, -0.10], color='#94a3b8', lw=1.4)

    # -------------------------------------------------------------------------
    # 1. Wheels (Front & Rear)
    # -------------------------------------------------------------------------
    for xw in [x_front, x_rear]:
        # Tire
        wheel = plt.Circle((xw, r_wheel), r_wheel, facecolor='#1e293b', edgecolor='#0f172a', lw=3, zorder=3)
        # Rim
        rim = plt.Circle((xw, r_wheel), r_wheel * 0.54, facecolor='#94a3b8', edgecolor='#334155', lw=2, zorder=4)
        # Wheel Hub
        hub = plt.Circle((xw, r_wheel), r_wheel * 0.18, facecolor='#e2e8f0', edgecolor='#0f172a', lw=1.5, zorder=5)
        ax.add_patch(wheel)
        ax.add_patch(rim)
        ax.add_patch(hub)

    # -------------------------------------------------------------------------
    # 2. Vehicle Body Outline (Pickup Truck Silhouette)
    # Underbody is completely flat/level (ท้องรถเสมอราบไป)
    # No window rectangle (ไม่มีสี่เหลี่ยมกระจก)
    # Wheel arches provide realistic clearance over tires
    # -------------------------------------------------------------------------
    r_arch = 0.52
    theta_front = np.linspace(np.pi - 0.15, 0.15, 25)
    f_arch_x = (x_front + r_arch * np.cos(theta_front)).tolist()
    f_arch_y = (r_wheel + r_arch * np.sin(theta_front)).tolist()

    theta_rear = np.linspace(np.pi - 0.15, 0.15, 25)
    r_arch_x = (x_rear + r_arch * np.cos(theta_rear)).tolist()
    r_arch_y = (r_wheel + r_arch * np.sin(theta_rear)).tolist()

    body_pts = []
    # Front bumper & lower grille
    body_pts.append([0.10, y_belly])
    body_pts.append([0.10, 1.08])
    # Hood slope
    body_pts.append([0.80, 1.15])
    # Windshield A-pillar
    body_pts.append([1.45, 1.75])
    # Cab roof
    body_pts.append([2.65, 1.75])
    # Cab rear window drop to truck bed
    body_pts.append([2.70, 1.22])
    # Truck cargo bed upper rail
    body_pts.append([4.65, 1.22])
    # Tailgate / rear bumper drop
    body_pts.append([4.65, y_belly])
    # Rear overhang flat sill to rear wheel arch
    body_pts.append([r_arch_x[-1], y_belly])
    # Rear wheel arch cutout (reversed so it traces counter-clockwise / cleanly)
    for px, py in zip(reversed(r_arch_x), reversed(r_arch_y)):
        body_pts.append([px, py])
    body_pts.append([r_arch_x[0], y_belly])
    # Belly / Rocker sill line between front & rear wheels (Completely Flat & Level)
    body_pts.append([f_arch_x[-1], y_belly])
    # Front wheel arch cutout
    for px, py in zip(reversed(f_arch_x), reversed(f_arch_y)):
        body_pts.append([px, py])
    body_pts.append([f_arch_x[0], y_belly])
    # Front overhang flat sill to front bumper
    body_pts.append([0.10, y_belly])

    body_poly = patches.Polygon(body_pts, closed=True, facecolor='#e2e8f0', edgecolor='#334155', lw=2.5, zorder=2)
    ax.add_patch(body_poly)

    # -------------------------------------------------------------------------
    # 3. Leaf Spring Assembly (Suspension at Rear Axle, Mounted to Chassis Frame)
    # Longer span, arching upwards to mount directly to the chassis frame (โครงรถ)
    # -------------------------------------------------------------------------
    ls_span = 0.65
    y_center = 0.38          # Axle center height
    ls_camber = 0.065        # Upward curvature to meet vehicle underbody sill
    ls_x = np.linspace(x_rear - ls_span, x_rear + ls_span, 80)
    # Master leaf curves upwards from axle to vehicle underbody
    ls_y_master = y_center + ls_camber * ((ls_x - x_rear) / ls_span)**2
    ax.plot(ls_x, ls_y_master, color='#dc2626', lw=4.5, zorder=6, label='Leaf Spring Assembly')

    # Secondary graduated leaf
    ls_span_sub = 0.46
    ls_x_sub = np.linspace(x_rear - ls_span_sub, x_rear + ls_span_sub, 50)
    ls_y_sub = (y_center - 0.018) + (ls_camber * 0.90) * ((ls_x_sub - x_rear) / ls_span)**2
    ax.plot(ls_x_sub, ls_y_sub, color='#991b1b', lw=3.2, zorder=6)

    # Third graduated leaf
    ls_span_sub3 = 0.28
    ls_x_sub3 = np.linspace(x_rear - ls_span_sub3, x_rear + ls_span_sub3, 30)
    ls_y_sub3 = (y_center - 0.034) + (ls_camber * 0.80) * ((ls_x_sub3 - x_rear) / ls_span)**2
    ax.plot(ls_x_sub3, ls_y_sub3, color='#7f1d1d', lw=2.2, zorder=6)

    # Spring Eyes (Front Fixed Eye & Rear Shackle Eye) mounting to vehicle underbody
    y_front_eye = ls_y_master[0]
    y_rear_eye = ls_y_master[-1]
    ax.scatter([x_rear - ls_span], [y_front_eye], color='#991b1b', edgecolor='#0f172a', s=75, lw=1.6, zorder=7)
    ax.scatter([x_rear + ls_span], [y_rear_eye], color='#991b1b', edgecolor='#0f172a', s=75, lw=1.6, zorder=7)

    # Axle Clamp / U-Bolt Saddle Block on Rear Axle
    axle_clamp = patches.Rectangle((x_rear - 0.08, y_center - 0.04), 0.16, 0.08,
                                   facecolor='#334155', edgecolor='#0f172a', lw=1.5, zorder=7)
    ax.add_patch(axle_clamp)

    # Leaf Spring Annotation
    ax.annotate('Rear Leaf Spring Assembly\n(Subject FEA Component)',
                xy=(x_rear - 0.32, y_center + 0.02), xytext=(x_rear - 1.05, 0.90),
                fontsize=10.5, fontweight='bold', color='#991b1b',
                arrowprops=dict(arrowstyle='->', color='#dc2626', lw=1.8, connectionstyle='arc3,rad=-0.12'),
                bbox=dict(boxstyle='round,pad=0.4', facecolor='#fef2f2', edgecolor='#f87171', lw=1.2), zorder=9)

    # -------------------------------------------------------------------------
    # 4. Center of Gravity (CG)
    # No cluttered description box on CG
    # Height scale projected with dashed line to vehicle front with vertical arrow
    # -------------------------------------------------------------------------
    # Clean CG Quadrant Symbol
    r_cg = 0.085
    cg_circle = plt.Circle((x_cg, y_cg), r_cg, facecolor='#f59e0b', edgecolor='#78350f', lw=2.2, zorder=7)
    ax.add_patch(cg_circle)
    w1 = patches.Wedge((x_cg, y_cg), r_cg, 0, 90, facecolor='#78350f', zorder=8)
    w2 = patches.Wedge((x_cg, y_cg), r_cg, 180, 270, facecolor='#78350f', zorder=8)
    ax.add_patch(w1)
    ax.add_patch(w2)
    ax.text(x_cg, y_cg + 0.11, 'CG', ha='center', va='bottom', fontsize=11, fontweight='bold', color='#78350f', zorder=9)

    # Horizontal Dashed Projection Line to Front of Vehicle
    x_proj_front = -0.32
    ax.plot([x_proj_front, x_cg], [y_cg, y_cg], color='#b45309', lw=1.6, ls='--', zorder=6)
    # Ground reference dashed line out front
    ax.plot([x_proj_front - 0.15, x_proj_front + 0.12], [0, 0], color='#334155', lw=2, zorder=6)

    # Vertical Height Dimension Arrow at Front
    ax.annotate('', xy=(x_proj_front, y_cg), xytext=(x_proj_front, 0),
                arrowprops=dict(arrowstyle='<->', color='#b45309', lw=2.0), zorder=8)
    ax.text(x_proj_front - 0.06, y_cg / 2, 'CG Height\n$h_{\\mathrm{cg}} = 0.750\\text{ m}$',
            ha='right', va='center', fontsize=10.5, fontweight='bold', color='#78350f',
            bbox=dict(boxstyle='round,pad=0.35', facecolor='#fef3c7', edgecolor='#f59e0b', lw=1.2), zorder=9)

    # -------------------------------------------------------------------------
    # 5. Vehicle Forces & Reaction Vectors
    # -------------------------------------------------------------------------
    # Total Weight Vector W at CG
    ax.annotate('', xy=(x_cg, 0.12), xytext=(x_cg, y_cg),
                arrowprops=dict(facecolor='#d97706', edgecolor='#92400e', width=3.5, headwidth=11), zorder=8)
    # Total Weight text box placed IN FRONT OF (to the left of) the downward arrow
    ax.text(x_cg - 0.14, 0.42, 'Total Weight $W = M \\cdot g$\n$W = 27,468.0\\text{ N}$\n(GVW = 2,800 kg)',
            ha='right', va='center', fontsize=10.5, fontweight='bold', color='#92400e',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='#fffbeb', edgecolor='#fcd34d', lw=1.2), zorder=9)

    # Front Axle Reaction R_f
    ax.annotate('', xy=(x_front, 1.45), xytext=(x_front, 0),
                arrowprops=dict(facecolor='#2563eb', edgecolor='#1d4ed8', width=3.5, headwidth=11), zorder=8)
    ax.text(x_front - 0.76, 1.55, 'Front Axle Reaction $R_f$\n$R_f = 10,987.2\\text{ N}$\n(40% Static Load)',
            fontsize=10.5, fontweight='bold', color='#1e40af',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='#eff6ff', edgecolor='#93c5fd', lw=1.2), zorder=9)

    # Rear Axle Reaction R_r
    ax.annotate('', xy=(x_rear, 1.55), xytext=(x_rear, 0),
                arrowprops=dict(facecolor='#16a34a', edgecolor='#15803d', width=4, headwidth=12), zorder=8)
    ax.text(x_rear - 0.70, 1.65,
            'Rear Axle Reaction $R_r = 16,480.8\\text{ N}$ (60% Total)\n'
            '↳ Static per spring: $F_{v,\\mathrm{static}} = 8,240.4\\text{ N}$\n'
            '↳ 2.0G Dynamic bump: $F_{v,\\mathrm{bump}} = 16,480.8\\text{ N}$',
            fontsize=10.5, fontweight='bold', color='#14532d',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='#f0fdf4', edgecolor='#86efac', lw=1.5), zorder=9)

    # -------------------------------------------------------------------------
    # 6. Horizontal Dimension Lines (Wheelbase L_wb, a, b)
    # -------------------------------------------------------------------------
    y_dim1 = -0.38
    ax.annotate('', xy=(x_rear, y_dim1), xytext=(x_front, y_dim1),
                arrowprops=dict(arrowstyle='<->', color='#1e293b', lw=1.8))
    ax.plot([x_front, x_front], [-0.05, y_dim1 - 0.10], color='#64748b', lw=1.2, ls='--')
    ax.plot([x_rear, x_rear], [-0.05, y_dim1 - 0.10], color='#64748b', lw=1.2, ls='--')
    ax.text((x_front + x_rear) / 2, y_dim1 + 0.08, 'Wheelbase $L_{\\mathrm{wb}} = 3.085\\text{ m}$',
            ha='center', va='bottom', fontsize=11.5, fontweight='bold', color='#0f172a',
            bbox=dict(boxstyle='round,pad=0.2', facecolor='#ffffff', edgecolor='#cbd5e1', lw=1))

    # Sub-dimensions a and b
    y_dim2 = -0.72
    ax.annotate('', xy=(x_cg, y_dim2), xytext=(x_front, y_dim2), arrowprops=dict(arrowstyle='<->', color='#475569', lw=1.5))
    ax.annotate('', xy=(x_rear, y_dim2), xytext=(x_cg, y_dim2), arrowprops=dict(arrowstyle='<->', color='#475569', lw=1.5))
    ax.plot([x_cg, x_cg], [0.12, y_dim2 - 0.10], color='#d97706', lw=1.2, ls=':')
    ax.plot([x_front, x_front], [y_dim1, y_dim2 - 0.10], color='#64748b', lw=1.2, ls='--')
    ax.plot([x_rear, x_rear], [y_dim1, y_dim2 - 0.10], color='#64748b', lw=1.2, ls='--')
    ax.text((x_front + x_cg) / 2, y_dim2 - 0.08, 'Front to CG: $a = 1.851\\text{ m}$',
            ha='center', va='top', fontsize=10, fontweight='bold', color='#334155')
    ax.text((x_cg + x_rear) / 2, y_dim2 - 0.08, 'CG to Rear: $b = 1.234\\text{ m}$',
            ha='center', va='top', fontsize=10, fontweight='bold', color='#334155')

    # -------------------------------------------------------------------------
    # 7. Figure Header & Title
    # -------------------------------------------------------------------------
    ax.text(0.10, 2.42, 'Vehicle Free Body Diagram & Suspension Load Distribution',
            fontsize=16, fontweight='bold', color='#0f172a')
    ax.text(0.10, 2.24, 'Light Commercial Vehicle (1-Tonne Laden Pickup Truck) | Total Gravitational Mass M = 2,800 kg',
            fontsize=11.5, color='#475569')

    ax.set_xlim(-0.85, 4.95)
    ax.set_ylim(-1.0, 2.60)
    ax.axis('off')
    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f'[OK] Generated: {out_path}')

if __name__ == '__main__':
    generate_vehicle_fbd()
