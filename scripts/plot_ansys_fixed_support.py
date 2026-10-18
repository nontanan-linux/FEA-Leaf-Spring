"""
Generate ANSYS Perspective Fixed Support Boundary Condition Visualization
Highlights the inner cylindrical surface of the leaf spring eye for Fixed Support (Encastre).
Produces:
  1. scripts/fixed_support_model.png (Front eye fixed support with zoom inset & full restraints)
  2. scripts/fixed_support_ansys.png (Alias of fixed_support_model.png)
  3. scripts/fixed_support_right_eye.png (Alternative configuration scoping the right eye)
"""

import os
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from collections import deque
from scipy.ndimage import binary_dilation

def generate_ansys_fixed_support_figures():
    scripts_dir = os.path.dirname(os.path.abspath(__file__))
    project_dir = os.path.dirname(scripts_dir)
    
    # Try finding the original ANSYS screenshot
    possible_paths = [
        os.path.join(r"C:\Users\Acer\.gemini\antigravity\brain\b9bbf4b0-df85-4b3a-a851-1c743a2c4158\.user_uploaded", "media_1792340880651.png"),
        os.path.join(scripts_dir, "ansys_raw_view.png"),
        os.path.join(project_dir, "resource", "Leaf Spring.jpg")
    ]
    
    base_img_path = None
    for p in possible_paths:
        if os.path.exists(p):
            base_img_path = p
            break
            
    if not base_img_path:
        raise FileNotFoundError("Base ANSYS screenshot could not be located.")
        
    orig = Image.open(base_img_path).convert('RGBA')
    w_orig, h_orig = orig.size
    
    scale = 2
    w, h = w_orig * scale, h_orig * scale
    
    # Standard Windows fonts
    font_bold = ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf', int(12 * scale))
    font_norm = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', int(11 * scale))
    font_small = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', int(9.5 * scale))
    font_zoom = ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf', int(9 * scale))
    
    # =========================================================================
    # FIGURE 1: Front Eye (Left Eye) Fixed Support with Scoped Detail & Shackle
    # =========================================================================
    img1 = orig.resize((w, h), Image.LANCZOS)
    arr1 = np.array(img1).copy()
    
    # 1. Segment Left Eye Hole
    seed_left = (int(40 * scale), int(64.5 * scale))
    visited = np.zeros((h, w), dtype=bool)
    mask_left = np.zeros((h, w), dtype=bool)
    q = deque([seed_left])
    visited[seed_left] = True
    while q:
        r, c = q.popleft()
        mask_left[r, c] = True
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if 22*scale <= nr < 65*scale and 51*scale <= nc < 78*scale and not visited[nr, nc]:
                visited[nr, nc] = True
                rgb = arr1[nr, nc, :3]
                gray = np.mean(rgb)
                if gray >= 80:
                    q.append((nr, nc))
                    
    # Highlight in ANSYS Cyan-Blue (#00BEFF) with metallic depth preservation
    highlight_color = np.array([0, 190, 255], dtype=np.float32)
    alpha = 0.72
    for r in range(h):
        for c in range(w):
            if mask_left[r, c]:
                val = arr1[r, c, :3].astype(np.float32)
                arr1[r, c, :3] = (1.0 - alpha) * val + alpha * highlight_color
                
    res1 = Image.fromarray(arr1)
    
    # Draw contour outline around scoped surface
    dilated = binary_dilation(mask_left, iterations=2)
    contour = dilated & (~mask_left)
    draw1 = ImageDraw.Draw(res1)
    for r in range(h):
        for c in range(w):
            if contour[r, c]:
                draw1.point((c, r), fill=(0, 245, 255, 255))
                
    # Extract zoom inset of left eye (before any vector lines)
    crop_center_x, crop_center_y = int(66.5 * scale), int(40.5 * scale)
    crop_r = int(22 * scale)
    zoom_crop = res1.crop((crop_center_x - crop_r, crop_center_y - crop_r, 
                           crop_center_x + crop_r, crop_center_y + crop_r))
    zoom_factor = 2.4
    zoom_size = int(crop_r * 2 * zoom_factor)
    zoomed_inset = zoom_crop.resize((zoom_size, zoom_size), Image.LANCZOS)
    
    # Decorate zoomed inset with 3D constraint arrows
    draw_zoom = ImageDraw.Draw(zoomed_inset)
    zx_c, zy_c = zoom_size // 2, zoom_size // 2
    arrow_l = int(18 * scale)
    # Ux arrow (Red)
    draw_zoom.line([(zx_c, zy_c), (zx_c + arrow_l, zy_c)], fill=(230, 20, 20, 255), width=int(2*scale))
    draw_zoom.polygon([(zx_c + arrow_l, zy_c - int(3*scale)), (zx_c + arrow_l + int(6*scale), zy_c), 
                       (zx_c + arrow_l, zy_c + int(3*scale))], fill=(230, 20, 20, 255))
    draw_zoom.text((zx_c + arrow_l + int(7*scale), zy_c - int(5*scale)), 'Ux=0', fill=(210, 0, 0, 255), font=font_zoom)
    # Uy arrow (Green)
    draw_zoom.line([(zx_c, zy_c), (zx_c, zy_c - arrow_l)], fill=(20, 170, 20, 255), width=int(2*scale))
    draw_zoom.polygon([(zx_c - int(3*scale), zy_c - arrow_l), (zx_c, zy_c - arrow_l - int(6*scale)), 
                       (zx_c + int(3*scale), zy_c - arrow_l)], fill=(20, 170, 20, 255))
    draw_zoom.text((zx_c - int(12*scale), zy_c - arrow_l - int(14*scale)), 'Uy=0', fill=(0, 130, 0, 255), font=font_zoom)
    # Uz arrow (Blue)
    draw_zoom.line([(zx_c, zy_c), (zx_c - int(arrow_l*0.7), zy_c + int(arrow_l*0.7))], fill=(20, 60, 240, 255), width=int(2*scale))
    draw_zoom.polygon([(zx_c - int(arrow_l*0.7) - int(2*scale), zy_c + int(arrow_l*0.7) - int(3*scale)), 
                       (zx_c - int(arrow_l*0.7) - int(5*scale), zy_c + int(arrow_l*0.7) + int(5*scale)), 
                       (zx_c - int(arrow_l*0.7) + int(3*scale), zy_c + int(arrow_l*0.7) + int(2*scale))], fill=(20, 60, 240, 255))
    draw_zoom.text((zx_c - int(arrow_l*0.7) - int(24*scale), zy_c + int(arrow_l*0.7) + int(2*scale)), 'Uz=0', fill=(0, 40, 200, 255), font=font_zoom)
    # Center white dot
    draw_zoom.ellipse([zx_c - int(3*scale), zy_c - int(3*scale), zx_c + int(3*scale), zy_c + int(3*scale)], 
                      fill=(255, 255, 255, 255), outline=(0, 0, 0, 255), width=1)

    # Inset Box on Left
    shadow_offset = 3 * scale
    inset_x = 18 * scale
    inset_y = 115 * scale
    inset_w = zoom_size + 16*scale
    inset_h = zoom_size + 42*scale
    
    draw1.rounded_rectangle([inset_x + shadow_offset, inset_y + shadow_offset, 
                            inset_x + inset_w + shadow_offset, inset_y + inset_h + shadow_offset], 
                           radius=6*scale, fill=(30, 40, 50, 70))
    draw1.rounded_rectangle([inset_x, inset_y, inset_x + inset_w, inset_y + inset_h], 
                           radius=6*scale, fill=(255, 255, 255, 248), outline=(0, 140, 220, 255), width=2*scale)
    draw1.rounded_rectangle([inset_x, inset_y, inset_x + inset_w, inset_y + 22*scale], 
                           radius=6*scale, fill=(0, 140, 220, 255))
    draw1.rectangle([inset_x, inset_y + 14*scale, inset_x + inset_w, inset_y + 22*scale], fill=(0, 140, 220, 255))
    draw1.text((inset_x + 8*scale, inset_y + 4*scale), 'Detail: Scoped Inner Surface (Zoom 2.4x)', fill=(255, 255, 255, 255), font=font_bold)
    
    res1.paste(zoomed_inset, (inset_x + 8*scale, inset_y + 26*scale))
    draw1.rectangle([inset_x + 8*scale, inset_y + 26*scale, inset_x + 8*scale + zoom_size, inset_y + 26*scale + zoom_size], 
                   outline=(0, 160, 230, 255), width=2*scale)
    draw1.text((inset_x + 8*scale, inset_y + 26*scale + zoom_size + 4*scale), 
              '1 Scoped Cylindrical Face (\u0393_front_eye)', fill=(60, 60, 60, 255), font=font_small)
              
    target_left_pt = (int(66.5 * scale), int(40.5 * scale))
    draw1.line([target_left_pt, (target_left_pt[0], inset_y - 2*scale)], fill=(0, 140, 220, 180), width=1*scale)
    
    # Callout Box [A] Fixed Support (Encastre)
    box_x = 145 * scale
    box_y = 20 * scale
    box_w = 225 * scale
    box_h = 82 * scale
    
    draw1.rounded_rectangle([box_x + shadow_offset, box_y + shadow_offset, 
                            box_x + box_w + shadow_offset, box_y + box_h + shadow_offset], 
                           radius=6*scale, fill=(30, 40, 50, 70))
    draw1.rounded_rectangle([box_x, box_y, box_x + box_w, box_y + box_h], 
                           radius=6*scale, fill=(255, 255, 255, 248), outline=(0, 102, 204, 255), width=2*scale)
    draw1.rounded_rectangle([box_x, box_y, box_x + box_w, box_y + 23*scale], 
                           radius=6*scale, fill=(0, 102, 204, 255))
    draw1.rectangle([box_x, box_y + 15*scale, box_x + box_w, box_y + 23*scale], fill=(0, 102, 204, 255))
    
    draw1.text((box_x + 10*scale, box_y + 4*scale), '[A] Fixed Support (Encastre)', fill=(255, 255, 255, 255), font=font_bold)
    draw1.text((box_x + 10*scale, box_y + 28*scale), 'Scope: 1 Face (Inner Bushing Bore)', fill=(30, 30, 30, 255), font=font_norm)
    draw1.text((box_x + 10*scale, box_y + 45*scale), 'Restraints: Ux = 0, Uy = 0, Uz = 0', fill=(0, 120, 30, 255), font=font_bold)
    draw1.text((box_x + 10*scale, box_y + 63*scale), 'Location: Front Eye (Pinned to Chassis)', fill=(90, 90, 90, 255), font=font_small)
    
    draw1.line([(box_x, box_y + 42*scale), (box_x - 38*scale, box_y + 42*scale), target_left_pt], 
              fill=(0, 102, 204, 255), width=2*scale)
    draw1.ellipse([target_left_pt[0]-4*scale, target_left_pt[1]-4*scale, target_left_pt[0]+4*scale, target_left_pt[1]+4*scale], 
                 fill=(0, 240, 255, 255), outline=(0, 102, 204, 255), width=2*scale)
                 
    # Callout Box [B] Shackle Support (Rear Eye)
    shackle_box_x = 645 * scale
    shackle_box_y = 230 * scale
    shackle_box_w = 205 * scale
    shackle_box_h = 65 * scale
    
    draw1.rounded_rectangle([shackle_box_x + shadow_offset, shackle_box_y + shadow_offset,
                            shackle_box_x + shackle_box_w + shadow_offset, shackle_box_y + shackle_box_h + shadow_offset],
                           radius=6*scale, fill=(30, 40, 50, 60))
    draw1.rounded_rectangle([shackle_box_x, shackle_box_y, shackle_box_x + shackle_box_w, shackle_box_y + shackle_box_h],
                           radius=6*scale, fill=(255, 255, 255, 248), outline=(100, 120, 140, 255), width=2*scale)
    draw1.rounded_rectangle([shackle_box_x, shackle_box_y, shackle_box_x + shackle_box_w, shackle_box_y + 22*scale],
                           radius=6*scale, fill=(100, 120, 140, 255))
    draw1.rectangle([shackle_box_x, shackle_box_y + 14*scale, shackle_box_x + shackle_box_w, shackle_box_y + 22*scale], fill=(100, 120, 140, 255))
    
    draw1.text((shackle_box_x + 10*scale, shackle_box_y + 4*scale), '[B] Shackle Support (Rear Eye)', fill=(255, 255, 255, 255), font=font_bold)
    draw1.text((shackle_box_x + 10*scale, shackle_box_y + 27*scale), 'Restraints: Uy = 0, Uz = 0', fill=(40, 40, 40, 255), font=font_norm)
    draw1.text((shackle_box_x + 10*scale, shackle_box_y + 45*scale), 'Longitudinal Motion (Ux): Free', fill=(190, 50, 0, 255), font=font_bold)
    
    target_right_pt = (int(818 * scale), int(330 * scale))
    draw1.line([(shackle_box_x + shackle_box_w//2, shackle_box_y + shackle_box_h), 
               (shackle_box_x + shackle_box_w//2, target_right_pt[1] - 12*scale), target_right_pt], 
              fill=(100, 120, 140, 255), width=2*scale)
    draw1.ellipse([target_right_pt[0]-3*scale, target_right_pt[1]-3*scale, target_right_pt[0]+3*scale, target_right_pt[1]+3*scale],
                 fill=(255, 140, 50, 255), outline=(100, 120, 140, 255), width=1*scale)
                 
    # Axle Load Callout [C]
    center_pt = (int(450 * scale), int(290 * scale))
    arrow_f_len = 45 * scale
    draw1.line([(center_pt[0], center_pt[1] - arrow_f_len), center_pt], fill=(220, 30, 30, 255), width=3*scale)
    draw1.polygon([(center_pt[0] - 6*scale, center_pt[1] - 12*scale), 
                  (center_pt[0] + 6*scale, center_pt[1] - 12*scale), 
                  center_pt], fill=(220, 30, 30, 255))
    draw1.rounded_rectangle([center_pt[0] - 75*scale, center_pt[1] - arrow_f_len - 24*scale, 
                            center_pt[0] + 75*scale, center_pt[1] - arrow_f_len], 
                           radius=4*scale, fill=(255, 255, 255, 240), outline=(220, 30, 30, 255), width=1*scale)
    draw1.text((center_pt[0] - 68*scale, center_pt[1] - arrow_f_len - 19*scale), 
              'Axle Load: Fv = -8,240 N', fill=(200, 0, 0, 255), font=font_bold)

    # Save Figure 1
    out1_path = os.path.join(scripts_dir, "fixed_support_model.png")
    out1_ansys = os.path.join(scripts_dir, "fixed_support_ansys.png")
    res1.save(out1_path, dpi=(300, 300))
    res1.save(out1_ansys, dpi=(300, 300))
    print(f"[OK] Saved primary fixed support figure to:\n  - {out1_path}\n  - {out1_ansys}")

    # =========================================================================
    # FIGURE 2: Alternative Configuration — Right Eye Fixed Support
    # =========================================================================
    img2 = orig.resize((w, h), Image.LANCZOS)
    arr2 = np.array(img2).copy()
    
    seed_right = (int(326 * scale), int(817 * scale))
    visited2 = np.zeros((h, w), dtype=bool)
    mask_right = np.zeros((h, w), dtype=bool)
    q2 = deque([seed_right])
    visited2[seed_right] = True
    while q2:
        r, c = q2.popleft()
        mask_right[r, c] = True
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if 310*scale <= nr < 345*scale and 805*scale <= nc < 835*scale and not visited2[nr, nc]:
                visited2[nr, nc] = True
                rgb = arr2[nr, nc, :3]
                gray = np.mean(rgb)
                is_green = (rgb[1] > 100 and rgb[0] < 50 and rgb[2] < 50)
                if gray >= 80 and not is_green:
                    q2.append((nr, nc))
                    
    for r in range(h):
        for c in range(w):
            if mask_right[r, c]:
                val = arr2[r, c, :3].astype(np.float32)
                arr2[r, c, :3] = (1.0 - alpha) * val + alpha * highlight_color
                
    res2 = Image.fromarray(arr2)
    dilated2 = binary_dilation(mask_right, iterations=2)
    contour2 = dilated2 & (~mask_right)
    draw2 = ImageDraw.Draw(res2)
    for r in range(h):
        for c in range(w):
            if contour2[r, c]:
                draw2.point((c, r), fill=(0, 245, 255, 255))
                
    box2_x = 620 * scale
    box2_y = 170 * scale
    box2_w = 230 * scale
    box2_h = 82 * scale
    
    draw2.rounded_rectangle([box2_x + shadow_offset, box2_y + shadow_offset, 
                            box2_x + box2_w + shadow_offset, box2_y + box2_h + shadow_offset], 
                           radius=6*scale, fill=(30, 40, 50, 70))
    draw2.rounded_rectangle([box2_x, box2_y, box2_x + box2_w, box2_y + box2_h], 
                           radius=6*scale, fill=(255, 255, 255, 248), outline=(0, 102, 204, 255), width=2*scale)
    draw2.rounded_rectangle([box2_x, box2_y, box2_x + box2_w, box2_y + 23*scale], 
                           radius=6*scale, fill=(0, 102, 204, 255))
    draw2.rectangle([box2_x, box2_y + 15*scale, box2_x + box2_w, box2_y + 23*scale], fill=(0, 102, 204, 255))
    
    draw2.text((box2_x + 10*scale, box2_y + 4*scale), '[A] Fixed Support (Encastre)', fill=(255, 255, 255, 255), font=font_bold)
    draw2.text((box2_x + 10*scale, box2_y + 28*scale), 'Scope: 1 Face (Inner Bushing Bore)', fill=(30, 30, 30, 255), font=font_norm)
    draw2.text((box2_x + 10*scale, box2_y + 45*scale), 'Restraints: Ux = 0, Uy = 0, Uz = 0', fill=(0, 120, 30, 255), font=font_bold)
    draw2.text((box2_x + 10*scale, box2_y + 63*scale), 'Location: Eye Cylindrical Hole Surface', fill=(90, 90, 90, 255), font=font_small)
    
    target_r_pt = (int(817 * scale), int(326 * scale))
    draw2.line([(box2_x + box2_w//2, box2_y + box2_h), (box2_x + box2_w//2, target_r_pt[1] - 15*scale), target_r_pt], 
              fill=(0, 102, 204, 255), width=2*scale)
    draw2.ellipse([target_r_pt[0]-4*scale, target_r_pt[1]-4*scale, target_r_pt[0]+4*scale, target_r_pt[1]+4*scale], 
                 fill=(0, 240, 255, 255), outline=(0, 102, 204, 255), width=2*scale)
                 
    out2_path = os.path.join(scripts_dir, "fixed_support_right_eye.png")
    res2.save(out2_path, dpi=(300, 300))
    print(f"[OK] Saved alternative right-eye fixed support figure to: {out2_path}")

if __name__ == "__main__":
    generate_ansys_fixed_support_figures()
