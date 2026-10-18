"""
Comprehensive FEA Mesh and Boundary Conditions Plotter for Leaf Spring
Generates:
1. mesh_model_3d.png: High-resolution 3D Isometric Mesh View
2. mesh_boundary_conditions.png: Multi-view plot with boundary conditions, restraints, and loads
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

def generate_mesh_plots():
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    stl_path = os.path.join(base_dir, "resource", "Leaf Spring.stl")
    scripts_dir = os.path.join(base_dir, "scripts")
    
    triangles = load_stl(stl_path)
    all_verts = triangles.reshape(-1, 3)
    min_b = all_verts.min(axis=0)
    max_b = all_verts.max(axis=0)
    
    # -------------------------------------------------------------
    # PLOT 1: Single High-Res Isometric 3D Mesh
    # -------------------------------------------------------------
    fig = plt.figure(figsize=(12, 6), dpi=300)
    ax = fig.add_subplot(111, projection='3d')
    
    mesh_col = Poly3DCollection(
        triangles, 
        facecolors='#70A1D7', 
        edgecolors='#1C3144', 
        linewidths=0.3, 
        alpha=0.9
    )
    ax.add_collection3d(mesh_col)
    
    ax.set_xlim([min_b[0] - 20, max_b[0] + 20])
    ax.set_ylim([min_b[1] - 20, max_b[1] + 20])
    ax.set_zlim([min_b[2] - 20, max_b[2] + 20])
    
    ax.set_title(f"Figure 1: 3D Finite Element Mesh Model of the Leaf Spring ({len(triangles)} Solid Elements)", 
                 fontsize=13, fontweight='bold', pad=15)
    ax.set_xlabel("X (Longitudinal, mm)", fontsize=10, labelpad=8)
    ax.set_ylabel("Y (Camber Height, mm)", fontsize=10, labelpad=8)
    ax.set_zlabel("Z (Width, mm)", fontsize=10, labelpad=8)
    ax.view_init(elev=22, azim=-55)
    ax.grid(True, linestyle=':', alpha=0.5)
    
    out1 = os.path.join(scripts_dir, "mesh_model_3d.png")
    plt.tight_layout()
    plt.savefig(out1, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"[OK] Saved {out1}")
    
    # -------------------------------------------------------------
    # PLOT 2: Composite View (Isometric Mesh + Boundary Conditions)
    # -------------------------------------------------------------
    fig2 = plt.figure(figsize=(14, 8), dpi=300)
    
    # Subplot 1: Isometric View
    ax1 = fig2.add_subplot(211, projection='3d')
    mesh_col1 = Poly3DCollection(
        triangles, facecolors='#A1C4FD', edgecolors='#192A56', linewidths=0.25, alpha=0.85
    )
    ax1.add_collection3d(mesh_col1)
    ax1.set_xlim([min_b[0] - 20, max_b[0] + 20])
    ax1.set_ylim([min_b[1] - 20, max_b[1] + 20])
    ax1.set_zlim([min_b[2] - 20, max_b[2] + 20])
    ax1.set_title("3D Finite Element Discretization (Solid Mesh)", fontsize=12, fontweight='bold')
    ax1.view_init(elev=20, azim=-65)
    ax1.grid(True, linestyle=':', alpha=0.4)
    
    # Subplot 2: Side 2D View with Annotated Restraints & Loads
    ax2 = fig2.add_subplot(212)
    # Project vertices to X-Y plane
    for tri in triangles:
        poly_2d = tri[:, :2] # X and Y
        ax2.fill(poly_2d[:, 0], poly_2d[:, 1], color='#82B1FF', edgecolor='#0D47A1', linewidth=0.3, alpha=0.7)
        
    x_center = (min_b[0] + max_b[0]) / 2.0
    y_center_top = max_b[1]
    
    # Draw Front Eye Pinned constraint (at min_b[0])
    ax2.plot(min_b[0], min_b[1], 'g^', markersize=14, label="Front Eye: Pinned Support (Ux=Uy=Uz=0)")
    ax2.annotate("Front Eye:\nPinned Support\n(Ux = Uy = Uz = 0)", 
                 xy=(min_b[0], min_b[1]), xytext=(min_b[0] - 40, min_b[1] - 60),
                 arrowprops=dict(facecolor='green', shrink=0.08, width=1.5, headwidth=6),
                 fontsize=9, fontweight='bold', color='green', ha='center')
                 
    # Draw Rear Eye Shackle/Roller constraint (at max_b[0])
    ax2.plot(max_b[0], min_b[1], 'mo', markersize=12, label="Rear Eye: Roller/Shackle Support (Uy=Uz=0)")
    ax2.annotate("Rear Eye:\nRoller / Shackle\n(Uy = Uz = 0, Ux free)", 
                 xy=(max_b[0], min_b[1]), xytext=(max_b[0] + 40, min_b[1] - 60),
                 arrowprops=dict(facecolor='purple', shrink=0.08, width=1.5, headwidth=6),
                 fontsize=9, fontweight='bold', color='purple', ha='center')
                 
    # Draw Axle Clamping Force (at x_center, pointing down)
    ax2.annotate("Axle Clamping Seat\nApplied Force: F_v\n(Static = 8,240 N / Bump = 16,481 N)", 
                 xy=(x_center, y_center_top), xytext=(x_center, y_center_top + 50),
                 arrowprops=dict(facecolor='red', shrink=0.08, width=2.5, headwidth=8),
                 fontsize=10, fontweight='bold', color='red', ha='center')
                 
    ax2.set_title("Side View: Finite Element Mesh, Restraints & Applied Loading", fontsize=12, fontweight='bold')
    ax2.set_xlabel("Longitudinal Span X (mm)", fontsize=10)
    ax2.set_ylabel("Vertical Camber Y (mm)", fontsize=10)
    ax2.set_ylim([min_b[1] - 90, max_b[1] + 90])
    ax2.set_xlim([min_b[0] - 120, max_b[0] + 120])
    ax2.grid(True, linestyle='--', alpha=0.5)
    ax2.legend(loc='lower center', ncol=2, frameon=True)
    
    out2 = os.path.join(scripts_dir, "mesh_boundary_conditions.png")
    plt.tight_layout()
    plt.savefig(out2, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"[OK] Saved {out2}")

if __name__ == "__main__":
    generate_mesh_plots()
