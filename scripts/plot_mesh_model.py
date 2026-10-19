"""
Figure 4 Generator: 3D Leaf Spring Finite Element Model - Boundary Conditions & Applied Load Vectors
Evaluates 3D CAD Solid Geometry (resource/Leaf Spring.stl) using Open3D and renders:
  1. Pure 3D CAD model with authentic metallic shading, clips, and highlighted restraint/load zones
  2. Vertical & longitudinal lines of action (แนวแรง) at center clamp and both eye supports
  3. Applied multi-axial operational forces (F_v, F_x) directly at the axle clamping seat
  4. Reaction force vectors and boundary condition restraints (Ux, Uy, Uz) at front and rear eyes
  5. Detailed engineering callout cards stating exact force magnitudes (เท่าไหร่) and mechanical functions (function)
  6. 3D Global Coordinate Triad (X: Longitudinal, Y: Camber/Vertical, Z: Width)

Output saved to: results/mesh_boundary_conditions.png
"""

import os
import sys
import open3d as o3d
import numpy as np
from PIL import Image, ImageDraw, ImageFont

def render_fig4():
    scripts_dir = os.path.dirname(os.path.abspath(__file__))
    project_dir = os.path.dirname(scripts_dir)
    stl_path = os.path.join(project_dir, "resource", "Leaf Spring.stl")
    results_dir = os.path.join(project_dir, "results")
    os.makedirs(results_dir, exist_ok=True)
    output_path = os.path.join(results_dir, "mesh_boundary_conditions.png")

    if not os.path.exists(stl_path):
        raise FileNotFoundError(f"CAD STL file not found at: {stl_path}")

    # -------------------------------------------------------------------------
    # 1. LOAD & RENDER 3D CAD MODEL VIA OPEN3D
    # -------------------------------------------------------------------------
    mesh = o3d.io.read_triangle_mesh(stl_path)
    center = mesh.get_center()
    mesh.translate(-center)

    v = np.asarray(mesh.vertices)
    # Metallic vertical shading gradient
    y_norm = (v[:, 1] - v[:, 1].min()) / (v[:, 1].max() - v[:, 1].min())
    base_r = 0.58 + 0.22 * y_norm
    base_g = 0.62 + 0.20 * y_norm
    base_b = 0.70 + 0.18 * y_norm
    colors = np.column_stack([base_r, base_g, base_b])

    # Center clamp zone (Amber orange)
    center_mask = (abs(v[:, 0]) < 48) & (v[:, 1] < -55)
    colors[center_mask] = np.array([0.90, 0.48, 0.22])

    # Front eye (Vibrant cyan)
    dist_fe = np.sqrt((v[:, 0] - (-370.8 - center[0]))**2 + (v[:, 1] - (25.7 - center[1]))**2)
    colors[dist_fe < 9] = np.array([0.05, 0.78, 0.95])

    # Rear eye (Vibrant purple)
    dist_re = np.sqrt((v[:, 0] - (287.4 - center[0]))**2 + (v[:, 1] - (25.4 - center[1]))**2)
    colors[dist_re < 9] = np.array([0.75, 0.35, 0.90])

    # Rebound clips (Brass gold)
    clip_mask = (abs(v[:, 0] - (-170)) < 15) | (abs(v[:, 0] - (170)) < 15)
    colors[clip_mask] = np.array([0.84, 0.74, 0.42])

    mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
    mesh.compute_vertex_normals()

    vis = o3d.visualization.Visualizer()
    vis.create_window(width=2400, height=1350, visible=False)
    vis.add_geometry(mesh)
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0.985, 0.988, 0.995])
    opt.light_on = True
    opt.mesh_show_wireframe = False

    ctr = vis.get_view_control()
    ctr.set_front([0.10, 0.30, 0.95])
    ctr.set_up([0.0, 0.95, -0.30])
    ctr.set_lookat([-20, -20, 0])
    ctr.set_zoom(0.40)

    vis.poll_events()
    vis.update_renderer()
    temp_dir = os.path.join(scripts_dir, "temp_render")
    os.makedirs(temp_dir, exist_ok=True)
    temp_raw = os.path.join(temp_dir, "fig4_raw_cad.png")
    vis.capture_screen_image(temp_raw)
    vis.destroy_window()

    # -------------------------------------------------------------------------
    # 2. COMPOSITE ANNOTATIONS, FORCES & BOUNDARY CONDITIONS
    # -------------------------------------------------------------------------
    img = Image.open(temp_raw).convert('RGBA')
    draw = ImageDraw.Draw(img)

    # Load Windows standard fonts
    font_title = ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf', 24)
    font_subtitle = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 15)
    font_head = ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf', 15)
    font_bold = ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf', 13)
    font_body = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 13)
    font_anno = ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf', 12)
    font_small = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 11)

    # Feature keypoints in 2400-rendered space
    p_fe = (292, 362)      # Front eye center
    p_re = (1679, 334)     # Rear eye center
    p_cl = (939, 510)      # Center clamp contact top

    # Title & Subtitle Banner
    draw.text((40, 25), 'Figure 4: Finite Element Loading & Boundary Conditions on 3D Leaf Spring Model', 
              fill=(15, 23, 42, 255), font=font_title)
    draw.text((40, 58), 'Applied Multi-Axial Operational Forces, Lines of Action, and Reaction Restraints on 3D Solid CAD Geometry', 
              fill=(71, 85, 105, 255), font=font_subtitle)

    # Helper: draw dashed line
    def draw_dashed_line(pt1, pt2, color, dash=9, space=7, width=2):
        x1, y1 = pt1
        x2, y2 = pt2
        dx, dy = x2 - x1, y2 - y1
        dist = np.hypot(dx, dy)
        if dist == 0: return
        ux, uy = dx / dist, dy / dist
        curr = 0
        while curr < dist:
            seg_end = min(curr + dash, dist)
            draw.line([(x1 + curr * ux, y1 + curr * uy), (x1 + seg_end * ux, y1 + seg_end * uy)], fill=color, width=width)
            curr += dash + space

    # Helper: draw vector arrow
    def draw_arrow(xy_from, xy_to, color, width=4, head_len=18, head_w=10):
        dx = xy_to[0] - xy_from[0]
        dy = xy_to[1] - xy_from[1]
        L = np.hypot(dx, dy)
        if L == 0: return
        ux, uy = dx / L, dy / L
        draw.line([xy_from, xy_to], fill=color, width=width)
        p1 = xy_to
        p2 = (xy_to[0] - head_len * ux + head_w * uy, xy_to[1] - head_len * uy - head_w * ux)
        p3 = (xy_to[0] - head_len * ux - head_w * uy, xy_to[1] - head_len * uy + head_w * ux)
        draw.polygon([p1, p2, p3], fill=color)

    # Helper: draw anchor ring
    def draw_anchor(pt, color, radius=7):
        draw.ellipse([pt[0]-radius, pt[1]-radius, pt[0]+radius, pt[1]+radius], 
                     fill=(255, 255, 255, 255), outline=color, width=3)

    # -------------------------------------------------------------------------
    # A. LINES OF ACTION (แนวแรง - Dashed Reference Tracks)
    # -------------------------------------------------------------------------
    # Vertical line of action at center clamp
    draw_dashed_line((p_cl[0], 120), (p_cl[0], 680), color=(220, 38, 38, 170), dash=10, space=7, width=2)
    # Longitudinal line of action at center clamp
    draw_dashed_line((680, p_cl[1]), (1240, p_cl[1]), color=(37, 99, 235, 170), dash=10, space=7, width=2)

    # Vertical reaction line of action at front eye
    draw_dashed_line((p_fe[0], 200), (p_fe[0], 580), color=(22, 163, 74, 170), dash=10, space=7, width=2)
    # Vertical reaction line of action at rear eye
    draw_dashed_line((p_re[0], 180), (p_re[0], 580), color=(147, 51, 234, 170), dash=10, space=7, width=2)

    # -------------------------------------------------------------------------
    # B. APPLIED FORCE VECTORS (AT CENTER AXLE CLAMP)
    # -------------------------------------------------------------------------
    # Vertical Force F_v (Red, pointing DOWN onto clamp surface)
    f_vert_start = (p_cl[0], 270)
    f_vert_end = (p_cl[0], p_cl[1] - 4)
    draw_arrow(f_vert_start, f_vert_end, color=(220, 38, 38, 255), width=6, head_len=22, head_w=12)

    # Label on F_v arrow
    draw.rounded_rectangle([p_cl[0] + 12, 380, p_cl[0] + 285, 420], radius=5, fill=(255, 255, 255, 235), outline=(220, 38, 38, 255), width=1)
    draw.text((p_cl[0] + 20, 385), 'Vertical Ride Load (F_v)', fill=(185, 28, 28, 255), font=font_bold)
    draw.text((p_cl[0] + 20, 401), 'F_v = -8,240.4 N (1G) / -16,480.8 N (2G)', fill=(71, 85, 105, 255), font=font_small)

    # Longitudinal Braking Force F_x (Blue, pointing FORWARD / LEFT)
    f_horiz_start = (p_cl[0] + 200, p_cl[1])
    f_horiz_end = (p_cl[0] + 16, p_cl[1])
    draw_arrow(f_horiz_start, f_horiz_end, color=(37, 99, 235, 255), width=5, head_len=20, head_w=10)

    # Label on F_x arrow
    draw.rounded_rectangle([p_cl[0] + 45, p_cl[1] + 16, p_cl[0] + 295, p_cl[1] + 54], radius=5, fill=(255, 255, 255, 235), outline=(37, 99, 235, 255), width=1)
    draw.text((p_cl[0] + 52, p_cl[1] + 20), 'Braking Traction Force (F_x)', fill=(29, 78, 216, 255), font=font_bold)
    draw.text((p_cl[0] + 52, p_cl[1] + 36), 'F_x = -3,500.0 N (Brake) / -5,000 N (Max)', fill=(71, 85, 105, 255), font=font_small)

    # Center anchor ring
    draw_anchor(p_cl, (220, 38, 38, 255), radius=7)

    # -------------------------------------------------------------------------
    # C. REACTION FORCES & RESTRAINTS (FRONT & REAR EYES)
    # -------------------------------------------------------------------------
    # Front Eye Reactions:
    # R_y1 (Green, pointing UP into eye)
    draw_arrow((p_fe[0], 500), (p_fe[0], p_fe[1] + 12), color=(22, 163, 74, 255), width=5, head_len=18, head_w=9)
    draw.text((p_fe[0] + 12, 450), 'R_y1 = +4,120 N (1G)\n        +8,240 N (2G)', fill=(22, 163, 74, 255), font=font_bold)

    # R_x1 (Green, pointing RIGHT resisting brake)
    draw_arrow((p_fe[0] - 120, p_fe[1]), (p_fe[0] - 14, p_fe[1]), color=(22, 163, 74, 255), width=4, head_len=16, head_w=8)
    draw.text((p_fe[0] - 165, p_fe[1] - 22), 'R_x1 = +3,500 N', fill=(22, 163, 74, 255), font=font_bold)

    # Front eye anchor ring
    draw_anchor(p_fe, (22, 163, 74, 255), radius=7)

    # Rear Eye Reactions:
    # R_y2 (Purple, pointing UP into eye)
    draw_arrow((p_re[0], 480), (p_re[0], p_re[1] + 12), color=(147, 51, 234, 255), width=5, head_len=18, head_w=9)
    draw.text((p_re[0] + 14, 450), 'R_y2 = +4,120 N (1G)\n        +8,240 N (2G)', fill=(147, 51, 234, 255), font=font_bold)

    # Horizontal sliding compliance (+/- Delta X)
    draw_arrow((p_re[0] - 55, p_re[1]), (p_re[0] - 14, p_re[1]), color=(147, 51, 234, 255), width=3, head_len=13, head_w=7)
    draw_arrow((p_re[0] + 55, p_re[1]), (p_re[0] + 14, p_re[1]), color=(147, 51, 234, 255), width=3, head_len=13, head_w=7)
    draw.text((p_re[0] - 65, p_re[1] - 22), 'Ux = Free (Sliding)', fill=(147, 51, 234, 255), font=font_bold)

    # Rear eye anchor ring
    draw_anchor(p_re, (147, 51, 234, 255), radius=7)

    # -------------------------------------------------------------------------
    # D. ENGINEERING CALLOUT CARDS (VALUES & FUNCTIONS)
    # -------------------------------------------------------------------------
    def draw_card(box, header_text, lines, header_bg, border_color):
        bx, by, bw, bh = box
        # Drop shadow
        draw.rounded_rectangle([bx+4, by+4, bx+bw+4, by+bh+4], radius=8, fill=(180, 190, 205, 90))
        # Card background & border
        draw.rounded_rectangle([bx, by, bx+bw, by+bh], radius=8, fill=(255, 255, 255, 250), outline=border_color, width=2)
        # Header bar
        h_height = 32
        draw.rounded_rectangle([bx, by, bx+bw, by+h_height], radius=8, fill=header_bg)
        draw.rectangle([bx, by+h_height-8, bx+bw, by+h_height], fill=header_bg)
        draw.text((bx+12, by+6), header_text, fill=(255, 255, 255, 255), font=font_head)
        # Content lines
        cur_y = by + h_height + 8
        for prefix, val, bold, col in lines:
            if bold:
                draw.text((bx+12, cur_y), prefix, fill=col, font=font_bold)
                w_p = draw.textlength(prefix, font=font_bold)
                draw.text((bx+12+w_p, cur_y), val, fill=(30, 41, 59, 255), font=font_body)
            else:
                draw.text((bx+12, cur_y), prefix + val, fill=col, font=font_body)
            cur_y += 20

    # Center Load Card
    c_lines = [
        ('Vertical Force (F_v): ', 'Axle payload & dynamic road impact', True, (220, 38, 38, 255)),
        ('  - Static Laden (1G): ', 'F_v,static = -8,240.40 N', False, (71, 85, 105, 255)),
        ('  - Dynamic Bump (2G): ', 'F_v,bump   = -16,480.80 N (Peak design load)', False, (71, 85, 105, 255)),
        ('Longitudinal Force (F_x): ', 'Braking traction & axle wind-up', True, (37, 99, 235, 255)),
        ('  - Dynamic Braking: ', 'F_x,brake  = -3,500.00 N', False, (71, 85, 105, 255)),
        ('  - Severe Combined: ', 'F_x,max    = -5,000.00 N (Emergency stop)', False, (71, 85, 105, 255)),
        ('Function: ', 'Applies combined vertical ride and braking torque to leaf pack', False, (15, 23, 42, 255)),
    ]
    draw_card((710, 95, 460, 185), 'Axle Clamping Seat Loading (F_v, F_x)', c_lines, 
              header_bg=(220, 38, 38, 255), border_color=(220, 38, 38, 255))
    draw.line([(939, 280), (939, 310)], fill=(220, 38, 38, 255), width=2)

    # Front Eye Card
    fe_lines = [
        ('Restraints: ', 'Ux = 0, Uy = 0, Uz = 0 (Fixed / Encastre)', True, (22, 163, 74, 255)),
        ('Reaction Forces: ', 'Reacts axle load and braking thrust', True, (22, 163, 74, 255)),
        ('  - Static Vertical: ', 'R_y1 = +4,120.20 N', False, (71, 85, 105, 255)),
        ('  - Dynamic Bump: ', 'R_y1,bump = +8,240.40 N', False, (71, 85, 105, 255)),
        ('  - Braking Reaction: ', 'R_x1 = +3,500.00 N', False, (71, 85, 105, 255)),
        ('Function: ', 'Rigid chassis hanger mount; transmits drive/brake thrust to frame', False, (15, 23, 42, 255)),
    ]
    draw_card((50, 95, 430, 165), 'Front Eye: Pinned Support Restraint', fe_lines,
              header_bg=(22, 163, 74, 255), border_color=(22, 163, 74, 255))
    draw.line([(292, 260), (292, 335)], fill=(22, 163, 74, 255), width=2)

    # Rear Eye Card
    re_lines = [
        ('Restraints: ', 'Uy = 0, Uz = 0 | Ux = Free (Sliding Shackle)', True, (147, 51, 234, 255)),
        ('Reaction & Travel: ', 'Accommodates leaf flattening', True, (147, 51, 234, 255)),
        ('  - Static Vertical: ', 'R_y2 = +4,120.20 N', False, (71, 85, 105, 255)),
        ('  - Dynamic Bump: ', 'R_y2,bump = +8,240.40 N', False, (71, 85, 105, 255)),
        ('  - Elongation Travel: ', 'Delta L approx 23.40 mm', False, (71, 85, 105, 255)),
        ('Function: ', 'Swinging shackle; allows longitudinal leaf compliance under load', False, (15, 23, 42, 255)),
    ]
    draw_card((1440, 95, 430, 165), 'Rear Eye: Shackle Roller Support', re_lines,
              header_bg=(147, 51, 234, 255), border_color=(147, 51, 234, 255))
    draw.line([(1679, 260), (1679, 310)], fill=(147, 51, 234, 255), width=2)

    # -------------------------------------------------------------------------
    # E. 3D COORDINATE REFERENCE TRIAD
    # -------------------------------------------------------------------------
    triad_x, triad_y = 120, 650
    t_len = 45
    draw_arrow((triad_x, triad_y), (triad_x + t_len, triad_y), (220, 38, 38, 255), width=3, head_len=10, head_w=5)
    draw.text((triad_x + t_len + 8, triad_y - 8), 'X (Longitudinal)', fill=(220, 38, 38, 255), font=font_bold)

    draw_arrow((triad_x, triad_y), (triad_x, triad_y - t_len), (22, 163, 74, 255), width=3, head_len=10, head_w=5)
    draw.text((triad_x - 12, triad_y - t_len - 18), 'Y (Camber / Vertical)', fill=(22, 163, 74, 255), font=font_bold)

    draw_arrow((triad_x, triad_y), (triad_x - int(t_len*0.6), triad_y + int(t_len*0.6)), (37, 99, 235, 255), width=3, head_len=10, head_w=5)
    draw.text((triad_x - int(t_len*0.6) - 50, triad_y + int(t_len*0.6) + 4), 'Z (Width)', fill=(37, 99, 235, 255), font=font_bold)

    # Crop extra bottom white space
    cropped = img.crop((0, 0, 1924, 730))
    cropped.save(output_path, dpi=(300, 300))
    print(f"[SUCCESS] Figure 4 successfully regenerated and saved to: {output_path}")

if __name__ == "__main__":
    render_fig4()
