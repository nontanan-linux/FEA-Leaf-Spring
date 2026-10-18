"""
Direct 3D CAD Geometry Evaluation and Fixed Support Boundary Condition Visualizer
Reads CAD model directly from resource/Leaf Spring.stl using Open3D and renders:
  1. 3D Assembly CAD View (Isometric perspective matching engineering suspension view)
  2. Front Eye Inner Cylindrical Bore Surface Scoped and Highlighted in Cyan (Fixed Support)
  3. Detail A: High-resolution 3x zoom on the scoped cylindrical surface with constraint vectors
  4. Rear Eye Shackle Support Callout (Ux: Free, Uy=0, Uz=0)
  5. Center Axle Clamping Load Vector (Fv = -8,240.4 N)
  6. Global 3D Coordinate Triad (X: Red, Y: Green, Z: Blue)

Outputs saved to:
  - scripts/fixed_support_model.png (Composite engineering drawing for result.md)
  - scripts/fixed_support_cad_view.png
  - scripts/fixed_support_cad_closeup.png
"""

import os
import open3d as o3d
import numpy as np
from PIL import Image, ImageDraw, ImageFont

def render_cad_fixed_support():
    scripts_dir = os.path.dirname(os.path.abspath(__file__))
    project_dir = os.path.dirname(scripts_dir)
    stl_path = os.path.join(project_dir, "resource", "Leaf Spring.stl")

    if not os.path.exists(stl_path):
        raise FileNotFoundError(f"CAD STL file not found at: {stl_path}")

    # Load 3D CAD mesh directly
    mesh = o3d.io.read_triangle_mesh(stl_path)
    mesh.compute_vertex_normals()
    center = mesh.get_center()
    mesh.translate(-center)

    vertices = np.asarray(mesh.vertices)
    # Center of left eye (Front eye) in translated coordinate space
    cx_t = -372.81 - center[0]
    cy_t = 25.66 - center[1]
    cz_t = 690.56 - center[2]

    # Calculate distance to cylinder axis (axis is along Z)
    dist_axis = np.sqrt((vertices[:, 0] - cx_t)**2 + (vertices[:, 1] - cy_t)**2)
    is_front_eye = vertices[:, 0] < (cx_t + 25.0)

    # Base colors: metallic silvery steel
    colors = np.ones((len(vertices), 3)) * np.array([0.76, 0.79, 0.85])
    # Rebound clips: brass gold
    clip_mask = (abs(vertices[:, 0] - (-170 - center[0])) < 22) | (abs(vertices[:, 0] - (170 - center[0])) < 22)
    colors[clip_mask] = np.array([0.84, 0.76, 0.46])
    # Highlight inner cylindrical hole surface in vibrant cyan (< 8.0mm radius)
    inner_bore_mask = is_front_eye & (dist_axis < 8.0)
    colors[inner_bore_mask] = np.array([0.0, 0.88, 1.0])
    mesh.vertex_colors = o3d.utility.Vector3dVector(colors)

    # Temporary directory for intermediate views
    scratch_dir = os.path.join(scripts_dir, "temp_render")
    os.makedirs(scratch_dir, exist_ok=True)
    main_path = os.path.join(scratch_dir, "cad_main_full.png")
    close_path = os.path.join(scratch_dir, "cad_close_full.png")

    # 1. Render Main Full Assembly View
    vis1 = o3d.visualization.Visualizer()
    vis1.create_window(width=1920, height=1080, visible=False)
    vis1.add_geometry(mesh)
    opt1 = vis1.get_render_option()
    opt1.background_color = np.asarray([0.96, 0.97, 0.99])
    opt1.light_on = True

    ctr1 = vis1.get_view_control()
    ctr1.set_front([0.35, 0.35, 0.85])
    ctr1.set_up([-0.35, 0.92, -0.15])
    ctr1.set_lookat([0, 0, 0])
    ctr1.set_zoom(0.38)

    vis1.poll_events()
    vis1.update_renderer()
    vis1.capture_screen_image(main_path)
    vis1.destroy_window()

    # 2. Render Close-up on Front Eye
    vis2 = o3d.visualization.Visualizer()
    vis2.create_window(width=700, height=700, visible=False)
    vis2.add_geometry(mesh)
    opt2 = vis2.get_render_option()
    opt2.background_color = np.asarray([0.96, 0.97, 0.99])
    opt2.light_on = True

    ctr2 = vis2.get_view_control()
    ctr2.set_front([0.35, 0.35, 0.85])
    ctr2.set_up([-0.35, 0.92, -0.15])
    ctr2.set_lookat([cx_t, cy_t, cz_t])
    ctr2.set_zoom(0.19)

    vis2.poll_events()
    vis2.update_renderer()
    vis2.capture_screen_image(close_path)
    vis2.destroy_window()

    # 3. Create Composite Technical Diagram with Annotations
    main_img = Image.open(main_path).convert('RGBA')
    close_img = Image.open(close_path).convert('RGBA')

    font_title = ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf', 24)
    font_subtitle = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 15)
    font_head = ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf', 16)
    font_body = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 14)
    font_bold = ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf', 14)
    font_small = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 12)
    font_zoom = ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf', 13)

    # Crop 320x320 from close_img
    cx, cy = 320, 360
    r_crop = 160
    crop_detail = close_img.crop((cx - r_crop, cy - r_crop, cx + r_crop, cy + r_crop))
    draw_d = ImageDraw.Draw(crop_detail)
    dc_x, dc_y = r_crop, r_crop

    # Red Ux arrow
    draw_d.line([(dc_x, dc_y), (dc_x + 35, dc_y)], fill=(220, 20, 20, 255), width=3)
    draw_d.polygon([(dc_x + 35, dc_y - 5), (dc_x + 45, dc_y), (dc_x + 35, dc_y + 5)], fill=(220, 20, 20, 255))
    draw_d.text((dc_x + 48, dc_y - 9), 'Ux=0', fill=(200, 0, 0, 255), font=font_zoom)

    # Green Uy arrow
    draw_d.line([(dc_x, dc_y), (dc_x, dc_y - 35)], fill=(20, 160, 20, 255), width=3)
    draw_d.polygon([(dc_x - 5, dc_y - 35), (dc_x, dc_y - 45), (dc_x + 5, dc_y - 35)], fill=(20, 160, 20, 255))
    draw_d.text((dc_x - 18, dc_y - 62), 'Uy=0', fill=(0, 140, 0, 255), font=font_zoom)

    # Blue Uz arrow
    draw_d.line([(dc_x, dc_y), (dc_x - 25, dc_y + 25)], fill=(20, 70, 230, 255), width=3)
    draw_d.polygon([(dc_x - 28, dc_y + 20), (dc_x - 35, dc_y + 35), (dc_x - 20, dc_y + 28)], fill=(20, 70, 230, 255))
    draw_d.text((dc_x - 65, dc_y + 28), 'Uz=0', fill=(0, 40, 210, 255), font=font_zoom)

    # White anchor dot
    draw_d.ellipse([dc_x - 5, dc_y - 5, dc_x + 5, dc_y + 5], fill=(255, 255, 255, 255), outline=(0, 0, 0, 255), width=2)

    # Crop canvas to height 960 to remove extra bottom whitespace
    canvas_h = 960
    composite = main_img.crop((0, 0, 1920, canvas_h))
    draw = ImageDraw.Draw(composite)

    # Title Header at Top
    title_box_x = 40
    title_box_y = 25
    draw.text((title_box_x, title_box_y), 'Automotive Leaf Spring CAD Model \u2014 Fixed Support Boundary Condition', fill=(20, 35, 60, 255), font=font_title)
    draw.text((title_box_x, title_box_y + 32), 'Direct 3D CAD Geometry Evaluation (resource/Leaf Spring.stl) | Pinned Chassis Mount & Shackle Interface', fill=(80, 95, 115, 255), font=font_subtitle)

    # Inset Box Position on Left
    inset_x = 40
    inset_y = 100
    inset_w = 340
    inset_h = 390
    shadow = 5

    draw.rounded_rectangle([inset_x + shadow, inset_y + shadow, inset_x + inset_w + shadow, inset_y + inset_h + shadow], 
                           radius=8, fill=(180, 190, 205, 120))
    draw.rounded_rectangle([inset_x, inset_y, inset_x + inset_w, inset_y + inset_h], 
                           radius=8, fill=(255, 255, 255, 250), outline=(0, 120, 215, 255), width=2)
    draw.rounded_rectangle([inset_x, inset_y, inset_x + inset_w, inset_y + 36], 
                           radius=8, fill=(0, 120, 215, 255))
    draw.rectangle([inset_x, inset_y + 26, inset_x + inset_w, inset_y + 36], fill=(0, 120, 215, 255))
    draw.text((inset_x + 12, inset_y + 8), 'Detail A: Scoped Fixed Support Hole', fill=(255, 255, 255, 255), font=font_head)

    composite.paste(crop_detail, (inset_x + 10, inset_y + 44))
    draw.rectangle([inset_x + 10, inset_y + 44, inset_x + 10 + 320, inset_y + 44 + 320], outline=(0, 160, 230, 255), width=2)
    draw.text((inset_x + 14, inset_y + 368), 'Inner Cylindrical Surface (\u0393_front_eye, r = 6.58 mm)', fill=(60, 70, 80, 255), font=font_small)

    # Leader line from Front Eye (450, 193) to Detail Inset Box
    target_hole = (450, 193)
    draw.line([target_hole, (inset_x + inset_w, inset_y + 120)], fill=(0, 140, 230, 255), width=2)
    draw.ellipse([target_hole[0]-6, target_hole[1]-6, target_hole[0]+6, target_hole[1]+6], 
                 fill=(0, 235, 255, 255), outline=(0, 100, 180, 255), width=2)

    # Callout Box [A]: Fixed Support Specification
    tag_a_x = 40
    tag_a_y = 510
    tag_a_w = 430
    tag_a_h = 130

    draw.rounded_rectangle([tag_a_x + shadow, tag_a_y + shadow, tag_a_x + tag_a_w + shadow, tag_a_y + tag_a_h + shadow], 
                       radius=8, fill=(180, 190, 205, 120))
    draw.rounded_rectangle([tag_a_x, tag_a_y, tag_a_x + tag_a_w, tag_a_y + tag_a_h], 
                       radius=8, fill=(255, 255, 255, 250), outline=(0, 102, 204, 255), width=2)
    draw.rounded_rectangle([tag_a_x, tag_a_y, tag_a_x + tag_a_w, tag_a_y + 34], 
                       radius=8, fill=(0, 102, 204, 255))
    draw.rectangle([tag_a_x, tag_a_y + 24, tag_a_x + tag_a_w, tag_a_y + 34], fill=(0, 102, 204, 255))

    draw.text((tag_a_x + 12, tag_a_y + 7), '[A] Fixed Support (Front Eye Restraint)', fill=(255, 255, 255, 255), font=font_head)
    draw.text((tag_a_x + 14, tag_a_y + 42), 'Restraints: Ux = 0, Uy = 0, Uz = 0 (Encastre)', fill=(0, 130, 30, 255), font=font_bold)
    draw.text((tag_a_x + 14, tag_a_y + 64), 'Scoped Entity: 1 Inner Cylindrical Face (Bore)', fill=(40, 50, 60, 255), font=font_body)
    draw.text((tag_a_x + 14, tag_a_y + 84), 'Boundary Type: Rigid Pinned Bushing mount to vehicle chassis', fill=(70, 80, 90, 255), font=font_small)
    draw.text((tag_a_x + 14, tag_a_y + 104), 'Degrees of Freedom: All 3 translation modes locked', fill=(90, 100, 110, 255), font=font_small)

    draw.line([(tag_a_x + tag_a_w // 2, tag_a_y), (tag_a_x + tag_a_w // 2, inset_y + inset_h)], fill=(0, 102, 204, 200), width=2)

    # Callout Box [B]: Shackle Support at Rear Eye
    tag_b_x = 1420
    tag_b_y = 470
    tag_b_w = 450
    tag_b_h = 130

    draw.rounded_rectangle([tag_b_x + shadow, tag_b_y + shadow, tag_b_x + tag_b_w + shadow, tag_b_y + tag_b_h + shadow], 
                           radius=8, fill=(180, 190, 205, 120))
    draw.rounded_rectangle([tag_b_x, tag_b_y, tag_b_x + tag_b_w, tag_b_y + tag_b_h], 
                           radius=8, fill=(255, 255, 255, 250), outline=(90, 110, 130, 255), width=2)
    draw.rounded_rectangle([tag_b_x, tag_b_y, tag_b_x + tag_b_w, tag_b_y + 34], 
                           radius=8, fill=(90, 110, 130, 255))
    draw.rectangle([tag_b_x, tag_b_y + 24, tag_b_x + tag_b_w, tag_b_y + 34], fill=(90, 110, 130, 255))

    draw.text((tag_b_x + 12, tag_b_y + 7), '[B] Shackle / Roller Support (Rear Eye)', fill=(255, 255, 255, 255), font=font_head)
    draw.text((tag_b_x + 14, tag_b_y + 42), 'Restraints: Uy = 0, Uz = 0 | Ux: Free to Slide', fill=(190, 60, 0, 255), font=font_bold)
    draw.text((tag_b_x + 14, tag_b_y + 64), 'Scoped Entity: 1 Inner Cylindrical Face (Rear Bore)', fill=(40, 50, 60, 255), font=font_body)
    draw.text((tag_b_x + 14, tag_b_y + 84), 'Function: Accommodates leaf flattening under deflection', fill=(70, 80, 90, 255), font=font_small)
    draw.text((tag_b_x + 14, tag_b_y + 104), 'Suspension Link: Pivoting shackle link pinned mount', fill=(90, 100, 110, 255), font=font_small)

    rear_hole = (1755, 665)
    draw.line([(tag_b_x + 200, tag_b_y + tag_b_h), (tag_b_x + 200, rear_hole[1] - 30), rear_hole], fill=(90, 110, 130, 255), width=2)
    draw.ellipse([rear_hole[0]-5, rear_hole[1]-5, rear_hole[0]+5, rear_hole[1]+5], 
                 fill=(255, 140, 40, 255), outline=(90, 110, 130, 255), width=2)

    # Callout [C]: Center Axle Load
    center_load = (1125, 745)
    arrow_len = 70
    draw.line([(center_load[0], center_load[1] - arrow_len), center_load], fill=(220, 25, 25, 255), width=4)
    draw.polygon([(center_load[0] - 8, center_load[1] - 18), 
                  (center_load[0] + 8, center_load[1] - 18), 
                  center_load], fill=(220, 25, 25, 255))

    load_tag_w = 260
    load_tag_h = 48
    draw.rounded_rectangle([center_load[0] - load_tag_w // 2, center_load[1] - arrow_len - load_tag_h, 
                            center_load[0] + load_tag_w // 2, center_load[1] - arrow_len], 
                           radius=6, fill=(255, 255, 255, 245), outline=(220, 25, 25, 255), width=2)
    draw.text((center_load[0] - load_tag_w // 2 + 12, center_load[1] - arrow_len - load_tag_h + 6), 
              'Axle Clamping Load (Fv)', fill=(180, 0, 0, 255), font=font_bold)
    draw.text((center_load[0] - load_tag_w // 2 + 12, center_load[1] - arrow_len - load_tag_h + 26), 
              'Fv = -8,240.4 N (Static Laden)', fill=(60, 60, 60, 255), font=font_small)

    # Draw 3D Triad at Bottom Right (1760, 880)
    triad_x, triad_y = 1760, 880
    t_len = 50
    draw.line([(triad_x, triad_y), (triad_x + t_len, triad_y)], fill=(220, 30, 30, 255), width=3)
    draw.polygon([(triad_x + t_len, triad_y - 4), (triad_x + t_len + 8, triad_y), (triad_x + t_len, triad_y + 4)], fill=(220, 30, 30, 255))
    draw.text((triad_x + t_len + 12, triad_y - 8), 'X (Span)', fill=(200, 0, 0, 255), font=font_bold)

    draw.line([(triad_x, triad_y), (triad_x, triad_y - t_len)], fill=(20, 160, 20, 255), width=3)
    draw.polygon([(triad_x - 4, triad_y - t_len), (triad_x, triad_y - t_len - 8), (triad_x + 4, triad_y - t_len)], fill=(20, 160, 20, 255))
    draw.text((triad_x - 10, triad_y - t_len - 22), 'Y (Height)', fill=(0, 140, 0, 255), font=font_bold)

    draw.line([(triad_x, triad_y), (triad_x - int(t_len * 0.7), triad_y + int(t_len * 0.7))], fill=(20, 70, 230, 255), width=3)
    draw.polygon([(triad_x - int(t_len * 0.7) - 4, triad_y + int(t_len * 0.7) - 4), 
                  (triad_x - int(t_len * 0.7) - 8, triad_y + int(t_len * 0.7) + 8), 
                  (triad_x - int(t_len * 0.7) + 4, triad_y + int(t_len * 0.7) + 4)], fill=(20, 70, 230, 255))
    draw.text((triad_x - int(t_len * 0.7) - 65, triad_y + int(t_len * 0.7) + 5), 'Z (Width)', fill=(0, 40, 210, 255), font=font_bold)

    # Save outputs to results directory
    results_dir = os.path.join(project_dir, 'results')
    os.makedirs(results_dir, exist_ok=True)
    out_model = os.path.join(results_dir, 'fixed_support_model.png')
    out_cad = os.path.join(results_dir, 'fixed_support_cad_view.png')
    crop_detail_path = os.path.join(results_dir, 'fixed_support_cad_closeup.png')

    composite.save(out_model, dpi=(300, 300))
    composite.save(out_cad, dpi=(300, 300))
    crop_detail.save(crop_detail_path, dpi=(300, 300))
    print('[OK] Saved pure CAD figures to:')
    print('  -', out_model)
    print('  -', out_cad)
    print('  -', crop_detail_path)

if __name__ == '__main__':
    render_cad_fixed_support()
