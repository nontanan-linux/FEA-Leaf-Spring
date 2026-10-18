"""
Generate Dedicated Fixed Support Visualization for Leaf Spring FEA
Creates a 3D isometric and detailed view showing Fixed Support boundary conditions
Saves to scripts/fixed_support_model.png
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import struct
import os

def load_stl(filename):
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

def generate_fixed_support_figure():
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    stl_path = os.path.join(base_dir, "resource", "Leaf Spring.stl")
    out_path = os.path.join(base_dir, "scripts", "fixed_support_model.png")
    
    triangles = load_stl(stl_path)
    all_verts = triangles.reshape(-1, 3)
    min_b = all_verts.min(axis=0)
    max_b = all_verts.max(axis=0)
    
    fig = plt.figure(figsize=(15, 7), dpi=300)
    
    # -------------------------------------------------------------
    # Left Subplot: Full 3D View with Fixed Support Callouts
    # -------------------------------------------------------------
    ax1 = fig.add_subplot(121, projection='3d')
    
    # Classify triangles: highlight fixed support zone (front eye) in green
    front_eye_threshold = min_b[0] + 70.0 # Front eye zone
    
    body_triangles = []
    fixed_triangles = []
    
    for tri in triangles:
        # Check if centroid is in front eye zone
        centroid = tri.mean(axis=0)
        if centroid[0] <= front_eye_threshold:
            fixed_triangles.append(tri)
        else:
            body_triangles.append(tri)
            
    # Draw body in steel blue
    if len(body_triangles) > 0:
        body_col = Poly3DCollection(
            body_triangles, facecolors='#90CAF9', edgecolors='#1565C0', linewidths=0.2, alpha=0.8
        )
        ax1.add_collection3d(body_col)
        
    # Draw fixed support region in bright green with distinct borders
    if len(fixed_triangles) > 0:
        fixed_col = Poly3DCollection(
            fixed_triangles, facecolors='#00E676', edgecolors='#004D40', linewidths=0.5, alpha=0.95
        )
        ax1.add_collection3d(fixed_col)
        
    # Draw 3D constraint symbols (green cones/spheres at front eye)
    eye_center_x = min_b[0] + 25.0
    eye_center_y = min_b[1] + 15.0
    eye_center_z = (min_b[2] + max_b[2]) / 2.0
    
    # Draw constraint axes arrows
    arrow_len = 50.0
    ax1.quiver(eye_center_x, eye_center_y, eye_center_z, -arrow_len, 0, 0, color='red', linewidth=2.5, arrow_length_ratio=0.3)
    ax1.quiver(eye_center_x, eye_center_y, eye_center_z, 0, -arrow_len, 0, color='green', linewidth=2.5, arrow_length_ratio=0.3)
    ax1.quiver(eye_center_x, eye_center_y, eye_center_z, 0, 0, -arrow_len, color='blue', linewidth=2.5, arrow_length_ratio=0.3)
    
    ax1.set_xlim([min_b[0] - 40, max_b[0] + 40])
    ax1.set_ylim([min_b[1] - 40, max_b[1] + 40])
    ax1.set_zlim([min_b[2] - 40, max_b[2] + 40])
    ax1.view_init(elev=24, azim=-60)
    ax1.set_title("(a) 3D Overview: Fixed Support Applied on Front Eye\n(All Translations Restrained: Ux = Uy = Uz = 0)", 
                  fontsize=11, fontweight='bold', pad=12)
    ax1.set_xlabel("X (Span, mm)", fontsize=9)
    ax1.set_ylabel("Y (Height, mm)", fontsize=9)
    ax1.set_zlabel("Z (Width, mm)", fontsize=9)
    ax1.grid(True, linestyle=':', alpha=0.4)
    
    # -------------------------------------------------------------
    # Right Subplot: Zoomed-in Close-up of Fixed Support Interface
    # -------------------------------------------------------------
    ax2 = fig.add_subplot(122, projection='3d')
    
    # Draw zoomed view of front eye
    if len(fixed_triangles) > 0:
        fixed_zoom_col = Poly3DCollection(
            fixed_triangles, facecolors='#69F0AE', edgecolors='#00796B', linewidths=0.4, alpha=0.9
        )
        ax2.add_collection3d(fixed_zoom_col)
        
    # Draw adjacent body triangles
    near_fixed_triangles = [tri for tri in body_triangles if tri.mean(axis=0)[0] <= (min_b[0] + 160.0)]
    if len(near_fixed_triangles) > 0:
        near_col = Poly3DCollection(
            near_fixed_triangles, facecolors='#E3F2FD', edgecolors='#90CAF9', linewidths=0.2, alpha=0.6
        )
        ax2.add_collection3d(near_col)
        
    ax2.set_xlim([min_b[0] - 15, min_b[0] + 130])
    ax2.set_ylim([min_b[1] - 20, min_b[1] + 80])
    ax2.set_zlim([min_b[2] - 15, max_b[2] + 15])
    ax2.view_init(elev=20, azim=-45)
    
    ax2.set_title("(b) Detailed View: Inner Bushing Sleeve Surface\n[Fixed Support / Encastre Boundary Condition]", 
                  fontsize=11, fontweight='bold', pad=12)
    ax2.set_xlabel("X (mm)", fontsize=9)
    ax2.set_ylabel("Y (mm)", fontsize=9)
    ax2.set_zlabel("Z (mm)", fontsize=9)
    ax2.grid(True, linestyle=':', alpha=0.4)
    
    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"[OK] Saved fixed support figure to: {out_path}")

if __name__ == "__main__":
    generate_fixed_support_figure()
