"""
===============================================================================
Automotive Leaf Spring - Boundary Conditions & Loading Schematic Generator
Produces Figure 4: results/mesh_boundary_conditions.png
===============================================================================
Accurately maps:
1. Front Eye: Pinned Support (Ux = Uy = Uz = 0) directly attached at eye center
2. Rear Eye: Shackle Roller Support (Uy = Uz = 0, Ux = Free) directly attached at eye center
3. Axle Clamping Seat: 100 x 60 mm clamping pad subjected to:
   - Downward Vertical Force Fv (-8,240.4 N static / -16,480.8 N 2.0G bump)
   - Longitudinal Braking Traction Fx (-3,500 N / -5,000 N)
4. Engineering dimensions: Eye-to-eye span, Camber height, Clamping seat length
===============================================================================
"""

import os
import struct
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Rectangle, Circle, Polygon

def load_stl_facets(filename):
    """Loads binary STL triangles."""
    with open(filename, 'rb') as f:
        f.read(80) # Header
        num_triangles = struct.unpack('<I', f.read(4))[0]
        triangles = []
        for _ in range(num_triangles):
            data = f.read(50)
            if len(data) < 50:
                break
            v = struct.unpack('<12f', data[:48])
            v1 = (v[3], v[4], v[5])
            v2 = (v[6], v[7], v[8])
            v3 = (v[9], v[10], v[11])
            triangles.append([v1, v2, v3])
    return np.array(triangles)

def generate_figure_4():
    scripts_dir = os.path.dirname(os.path.abspath(__file__))
    project_dir = os.path.dirname(scripts_dir)
    stl_path = os.path.join(project_dir, "resource", "Leaf Spring.stl")
    output_path = os.path.join(project_dir, "results", "mesh_boundary_conditions.png")

    if not os.path.exists(stl_path):
        raise FileNotFoundError(f"CAD STL not found at: {stl_path}")

    triangles = load_stl_facets(stl_path)

    # Key geometric locations from CAD STL:
    # Front Eye center (left): x = -370.8 mm, y = +25.7 mm
    # Rear Eye center (right): x = +287.4 mm, y = +25.4 mm
    # Axle Clamping Seat top surface: x in [-48, +48] mm, y = -66.9 mm
    x_fe, y_fe = -370.8, 25.7
    x_re, y_re = 287.4, 25.4
    x_clamp = 0.0
    y_clamp_top = -66.9
    y_clamp_bot = -98.7

    # Setup high-resolution figure (300 DPI)
    fig, ax = plt.subplots(figsize=(17, 10.2), dpi=300)
    fig.patch.set_facecolor('#ffffff')
    ax.set_facecolor('#f8fafc')

    # Draw 2D projected CAD mesh facets
    for tri in triangles:
        poly_2d = tri[:, :2] # X, Y
        cx = np.mean(poly_2d[:, 0])
        # Detect rebound clip
        if (abs(cx - (-170)) < 22) or (abs(cx - (170)) < 22):
            face_col = '#d97706'
            edge_col = '#92400e'
            alpha_val = 0.90
        elif abs(cx) < 48 and np.mean(poly_2d[:, 1]) < -65:
            # Axle clamping zone
            face_col = '#f97316'
            edge_col = '#c2410c'
            alpha_val = 0.85
        else:
            # Steel leaf plates
            face_col = '#94a3b8'
            edge_col = '#475569'
            alpha_val = 0.70

        poly_patch = Polygon(poly_2d, closed=True, facecolor=face_col, edgecolor=edge_col,
                             linewidth=0.25, alpha=alpha_val, zorder=3)
        ax.add_patch(poly_patch)

    # -------------------------------------------------------------------------
    # 1. FRONT EYE PINNED SUPPORT (Left: Ux = Uy = Uz = 0, Rot_z = Free)
    # -------------------------------------------------------------------------
    r_eye_inner = 6.58
    # Inner bore
    inner_fe = Circle((x_fe, y_fe), r_eye_inner, facecolor='#ffffff', edgecolor='#0f172a', lw=1.5, zorder=5)
    ax.add_patch(inner_fe)

    # Pinned support hinge bracket
    pin_y_base = y_fe - 55.0
    # Hinge triangle
    triangle_fe = Polygon([[x_fe, y_fe], [x_fe - 24, pin_y_base], [x_fe + 24, pin_y_base]],
                          closed=True, facecolor='#059669', edgecolor='#064e3b', lw=2.2, zorder=6)
    ax.add_patch(triangle_fe)

    # Ground base line & hatch
    ax.plot([x_fe - 38, x_fe + 38], [pin_y_base, pin_y_base], color='#064e3b', lw=2.8, zorder=6)
    for hx in np.linspace(x_fe - 32, x_fe + 26, 8):
        ax.plot([hx, hx + 8], [pin_y_base, pin_y_base - 10], color='#064e3b', lw=1.8, zorder=6)

    # Center pivot pin circle
    pin_circle_fe = Circle((x_fe, y_fe), 4.5, facecolor='#fbbf24', edgecolor='#78350f', lw=2.0, zorder=7)
    ax.add_patch(pin_circle_fe)

    # Constraint vector arrows around front eye
    ax.annotate('', xy=(x_fe + 32, y_fe), xytext=(x_fe, y_fe),
                arrowprops=dict(arrowstyle="->", color='#dc2626', lw=2.4), zorder=8)
    ax.text(x_fe + 36, y_fe, r'$U_x=0$', fontsize=11.0, fontweight='bold', color='#dc2626', va='center')

    ax.annotate('', xy=(x_fe, y_fe + 32), xytext=(x_fe, y_fe),
                arrowprops=dict(arrowstyle="->", color='#059669', lw=2.4), zorder=8)
    ax.text(x_fe, y_fe + 36, r'$U_y=0$', fontsize=11.0, fontweight='bold', color='#059669', ha='center')

    # Front Eye Callout Card (Carefully positioned to avoid overlap)
    fe_card = """[A] Front Eye: Pinned Support (Encastre)
• Scoped: Inner cylindrical bore (r = 6.58 mm)
• Constraints: Ux = 0, Uy = 0, Uz = 0 (Fixed)
• Rotation: Rot_z = Free (Pivoting hinge)
• Physical Mount: Rigid chassis frame hanger"""
    ax.text(x_fe + 35, 160.0, fe_card, fontsize=9.2, fontweight='bold', color='#064e3b',
            va='bottom', ha='center',
            bbox=dict(boxstyle="round,pad=0.45", fc="#ecfdf5", ec="#059669", lw=1.8), zorder=9)
    # Dashed connector
    ax.plot([x_fe + 35, x_fe], [160.0, y_fe + 25], color='#059669', lw=1.5, ls='--', zorder=8)

    # -------------------------------------------------------------------------
    # 2. REAR EYE SHACKLE / ROLLER SUPPORT (Right: Uy = Uz = 0, Ux = Free)
    # -------------------------------------------------------------------------
    inner_re = Circle((x_re, y_re), r_eye_inner, facecolor='#ffffff', edgecolor='#0f172a', lw=1.5, zorder=5)
    ax.add_patch(inner_re)

    # Shackle link arms
    shackle_len = 50.0
    shackle_angle = np.radians(22) # tilted shackle
    sh_x_bot = x_re + shackle_len * np.sin(shackle_angle)
    sh_y_bot = y_re - shackle_len * np.cos(shackle_angle)

    # Shackle link arm bar
    ax.plot([x_re, sh_x_bot], [y_re, sh_y_bot], color='#6b21a8', lw=5.0, solid_capstyle='round', zorder=6)
    # Top pin
    pin_re_top = Circle((x_re, y_re), 4.5, facecolor='#fbbf24', edgecolor='#78350f', lw=2.0, zorder=7)
    ax.add_patch(pin_re_top)
    # Bottom pin
    pin_re_bot = Circle((sh_x_bot, sh_y_bot), 4.0, facecolor='#fbbf24', edgecolor='#78350f', lw=2.0, zorder=7)
    ax.add_patch(pin_re_bot)

    # Roller base plate & rollers
    roller_y_base = sh_y_bot - 14.0
    r_roller = 5.2
    r1 = Circle((sh_x_bot - 11, roller_y_base + r_roller), r_roller, facecolor='#a855f7', edgecolor='#581c87', lw=1.6, zorder=6)
    r2 = Circle((sh_x_bot + 11, roller_y_base + r_roller), r_roller, facecolor='#a855f7', edgecolor='#581c87', lw=1.6, zorder=6)
    ax.add_patch(r1)
    ax.add_patch(r2)

    # Roller track
    ax.plot([sh_x_bot - 38, sh_x_bot + 38], [roller_y_base, roller_y_base], color='#581c87', lw=2.8, zorder=6)
    for hx in np.linspace(sh_x_bot - 32, sh_x_bot + 26, 8):
        ax.plot([hx, hx + 8], [roller_y_base, roller_y_base - 10], color='#581c87', lw=1.8, zorder=6)

    # Constraint vector arrows around rear eye
    ax.annotate('', xy=(x_re, y_re + 32), xytext=(x_re, y_re),
                arrowprops=dict(arrowstyle="->", color='#059669', lw=2.4), zorder=8)
    ax.text(x_re, y_re + 36, r'$U_y=0$', fontsize=11.0, fontweight='bold', color='#059669', ha='center')

    # Double-headed sliding arrow Ux = Free
    ax.annotate('', xy=(x_re + 36, y_re), xytext=(x_re - 36, y_re),
                arrowprops=dict(arrowstyle="<->", color='#9333ea', lw=2.6), zorder=8)
    ax.text(x_re + 42, y_re, r'$U_x = \mathbf{Free}$' + '\n(Sliding)', fontsize=10.2, fontweight='bold',
            color='#7e22ce', va='center')

    # Rear Eye Callout Card (Positioned to upper-right without overlap)
    re_card = """[B] Rear Eye: Shackle Roller Support (Expansion)
• Scoped: Inner cylindrical bore (r = 6.58 mm)
• Constraints: Uy = 0, Uz = 0 (Fixed in Y, Z)
• Longitudinal DOF: Ux = Free (Swinging link)
• Function: Absorbs leaf flattening (ΔL ≈ 23.4 mm)
• Physical Mount: Rear shackle swinging bracket"""
    ax.text(x_re - 35, 160.0, re_card, fontsize=9.2, fontweight='bold', color='#581c87',
            va='bottom', ha='center',
            bbox=dict(boxstyle="round,pad=0.45", fc="#faf5ff", ec="#9333ea", lw=1.8), zorder=9)
    # Dashed connector
    ax.plot([x_re - 35, x_re], [160.0, y_fe + 25], color='#9333ea', lw=1.5, ls='--', zorder=8)

    # -------------------------------------------------------------------------
    # 3. AXLE CLAMPING SEAT & MULTI-DIRECTIONAL LOADING (Center: x in [-48, 48])
    # -------------------------------------------------------------------------
    # Clamping top plate indication
    plate_w = 96.8
    plate_h = 10.0
    clamp_plate = Rectangle((x_clamp - plate_w / 2.0, y_clamp_top), plate_w, plate_h,
                            facecolor='#ea580c', edgecolor='#9a3412', lw=2.2, hatch='///', zorder=5)
    ax.add_patch(clamp_plate)

    # U-Bolt clamp indications
    u1_x = x_clamp - plate_w / 2.0 + 8.0
    u2_x = x_clamp + plate_w / 2.0 - 8.0
    ax.plot([u1_x, u1_x], [y_clamp_top + plate_h + 8, y_clamp_bot - 8], color='#1e293b', lw=3.2, ls=':', zorder=4)
    ax.plot([u2_x, u2_x], [y_clamp_top + plate_h + 8, y_clamp_bot - 8], color='#1e293b', lw=3.2, ls=':', zorder=4)

    # Vertical Force Vector F_v (Landing EXACTLY on the clamp plate)
    f_vert_start_y = y_clamp_top + 90.0
    f_vert_end_y = y_clamp_top + plate_h
    ax.annotate('', xy=(x_clamp, f_vert_end_y), xytext=(x_clamp, f_vert_start_y),
                arrowprops=dict(facecolor='#dc2626', edgecolor='#991b1b', width=4.5, headwidth=14, headlength=15),
                zorder=8)

    # Longitudinal Braking Traction Force F_x (Pointing forward, -X)
    f_horiz_start_x = x_clamp + 95.0
    f_horiz_end_x = x_clamp + 12.0
    f_horiz_y = y_clamp_top + plate_h / 2.0
    ax.annotate('', xy=(f_horiz_end_x, f_horiz_y), xytext=(f_horiz_start_x, f_horiz_y),
                arrowprops=dict(facecolor='#2563eb', edgecolor='#1d4ed8', width=3.8, headwidth=13, headlength=14),
                zorder=8)

    # Axle Loading Callout Box (Positioned in center, below top cards)
    axle_card = """[C] Axle Clamping Seat Loading (60 x 100 mm Contact Zone)
• Vertical Ride Force (F_v):
  ↳ Static Laden Ride: F_v,static = -8,240.40 N (1G)
  ↳ 2.0G Dynamic Bump: F_v,bump   = -16,480.80 N (Peak dynamic impact)
• Longitudinal Braking Force (F_x):
  ↳ Dynamic Braking:   F_x,brake  = -3,500.00 N (Axle wind-up moment)
  ↳ Severe Combined:   F_x,max    = -5,000.00 N
• Clamping Interface: Dual high-tensile U-bolts clamping leaf pack to axle"""
    ax.text(x_clamp, 35.0, axle_card, fontsize=9.4, fontweight='bold', color='#991b1b',
            va='bottom', ha='center',
            bbox=dict(boxstyle="round,pad=0.5", fc="#fef2f2", ec="#dc2626", lw=1.8), zorder=9)

    # -------------------------------------------------------------------------
    # 4. DIMENSIONS & DATUM LINES
    # -------------------------------------------------------------------------
    dim_color = '#334155'
    # Clamp plate dimension (Directly below clamp plate)
    y_dim_clamp = y_clamp_bot - 16.0
    ax.plot([x_clamp - plate_w / 2.0, x_clamp - plate_w / 2.0], [y_clamp_bot, y_dim_clamp - 6], color='#ea580c', lw=1.0, ls=':')
    ax.plot([x_clamp + plate_w / 2.0, x_clamp + plate_w / 2.0], [y_clamp_bot, y_dim_clamp - 6], color='#ea580c', lw=1.0, ls=':')
    ax.annotate('', xy=(x_clamp - plate_w / 2.0, y_dim_clamp), xytext=(x_clamp + plate_w / 2.0, y_dim_clamp),
                arrowprops=dict(arrowstyle="<->", color='#c2410c', lw=1.4), zorder=7)
    ax.text(x_clamp, y_dim_clamp - 4, 'Axle Clamp Seat $L_c = 100\\text{ mm}$',
            fontsize=8.8, fontweight='bold', color='#c2410c', ha='center', va='top')

    # Total Span Dimension (Placed cleanly below)
    y_dim_span = -150.0
    # Front eye projection down
    ax.plot([x_fe, x_fe], [y_fe - 60, y_dim_span - 12], color='#94a3b8', lw=1.2, ls=':')
    # Rear eye projection down
    ax.plot([x_re, x_re], [y_re - 60, y_dim_span - 12], color='#94a3b8', lw=1.2, ls=':')
    # Center projection down
    ax.plot([x_clamp, x_clamp], [y_dim_clamp - 18, y_dim_span - 12], color='#f97316', lw=1.2, ls=':')

    # Dimension line eye-to-eye
    ax.annotate('', xy=(x_fe, y_dim_span), xytext=(x_re, y_dim_span),
                arrowprops=dict(arrowstyle="<->", color=dim_color, lw=1.8), zorder=7)
    ax.text((x_fe + x_re) / 2.0, y_dim_span + 6, 'Total Eye-to-Eye Span $2L = 658.2\\text{ mm}$ (Semi-span $L = 329.1\\text{ mm}$)',
            fontsize=10.5, fontweight='bold', color=dim_color, ha='center', va='bottom',
            bbox=dict(boxstyle="round,pad=0.25", fc="#ffffff", ec="#cbd5e1", lw=1.0))

    # Camber Height Dimension (Right side)
    x_camber_dim = x_re + 80.0
    ax.plot([x_clamp + 55, x_camber_dim + 15], [y_clamp_top, y_clamp_top], color='#94a3b8', lw=1.2, ls=':')
    ax.plot([x_re + 25, x_camber_dim + 15], [y_re, y_re], color='#94a3b8', lw=1.2, ls=':')
    ax.annotate('', xy=(x_camber_dim, y_clamp_top), xytext=(x_camber_dim, y_re),
                arrowprops=dict(arrowstyle="<->", color='#b45309', lw=1.6), zorder=7)
    camber_h = abs(y_re - y_clamp_top)
    ax.text(x_camber_dim + 6, (y_re + y_clamp_top) / 2.0, f'Camber Height\n$h = {camber_h:.1f}\\text{{ mm}}$',
            fontsize=9.5, fontweight='bold', color='#b45309', va='center')

    # Coordinate Reference Triad (Bottom Left)
    triad_x, triad_y = -420.0, -115.0
    triad_len = 38.0
    ax.annotate('', xy=(triad_x + triad_len, triad_y), xytext=(triad_x, triad_y),
                arrowprops=dict(arrowstyle="->", color='#dc2626', lw=2.4))
    ax.text(triad_x + triad_len + 4, triad_y, 'X (Longitudinal)', fontsize=9.0, fontweight='bold', color='#dc2626', va='center')

    ax.annotate('', xy=(triad_x, triad_y + triad_len), xytext=(triad_x, triad_y),
                arrowprops=dict(arrowstyle="->", color='#16a34a', lw=2.4))
    ax.text(triad_x, triad_y + triad_len + 5, 'Y (Camber / Vertical)', fontsize=9.0, fontweight='bold', color='#16a34a', ha='center')

    # -------------------------------------------------------------------------
    # 5. HEADER & SUMMARY TABLE
    # -------------------------------------------------------------------------
    fig.suptitle("Finite Element Boundary Conditions & Operational Loading Schematic",
                 fontsize=15.5, fontweight='bold', color='#0f172a', y=0.975)
    plt.title("Automotive Semi-Elliptic Leaf Spring Assembly | Structural Constraints & Applied Multi-Load Vectors",
              fontsize=11.5, color='#475569', pad=12)

    # Summary table banner at very bottom
    cols = ['Boundary Region', 'Scoped Entity', 'Boundary Type', 'Degrees of Freedom (DOFs)', 'Operational Force / Function']
    col_widths = [0.14, 0.20, 0.14, 0.18, 0.34]
    table_data = [
        ['Front Eye Bore', 'Inner cylinder (r = 6.58 mm)', 'Pinned Support', 'Ux = Uy = Uz = 0 (Fixed)', 'Rigid chassis hanger mount; transfers longitudinal braking traction'],
        ['Rear Eye Bore', 'Inner cylinder (r = 6.58 mm)', 'Shackle / Roller', 'Uy = Uz = 0 | Ux = Free', 'Swinging shackle; accommodates camber flattening elongation (ΔL ≈ 23.4 mm)'],
        ['Axle Clamping Seat', 'Center plate (60 x 100 mm)', 'Applied External Force', 'Coupled translation', 'Fv = -8,240.4 N (Static) / -16,480.8 N (2G Bump); Fx = -3,500 N (Braking)']
    ]

    table = ax.table(cellText=table_data, colLabels=cols, colWidths=col_widths, loc='bottom',
                     cellLoc='center', bbox=[0.02, -0.32, 0.96, 0.17])
    table.auto_set_font_size(False)
    table.set_fontsize(8.3)
    for k, cell in table.get_celld().items():
        cell.set_edgecolor('#cbd5e1')
        if k[0] == 0:
            cell.set_facecolor('#e2e8f0')
            cell.set_text_props(weight='bold', color='#0f172a')
        else:
            cell.set_facecolor('#ffffff' if k[0] % 2 == 1 else '#f8fafc')
            cell.set_text_props(color='#1e293b')

    # Axes limits and formatting
    ax.set_xlim(-460, 420)
    ax.set_ylim(-180, 260)
    ax.set_xlabel("Longitudinal Span X (mm)", fontsize=10.5, fontweight='bold', color='#334155', labelpad=8)
    ax.set_ylabel("Vertical Camber Y (mm)", fontsize=10.5, fontweight='bold', color='#334155', labelpad=8)
    ax.grid(True, linestyle='--', alpha=0.35, color='#94a3b8')

    plt.subplots_adjust(top=0.91, bottom=0.26, left=0.07, right=0.95)
    plt.savefig(output_path, dpi=300)
    plt.close()
    print(f"[SUCCESS] Figure 4 successfully regenerated and saved to: {output_path}")

if __name__ == "__main__":
    generate_figure_4()
