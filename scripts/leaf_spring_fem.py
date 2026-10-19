"""
===============================================================================
Automotive Suspension Leaf Spring - Python Finite Element Method (FEM) Solver
Course: Finite Element Method in Engineering (M.Eng)
Instructor: Asst. Prof. Dr. Monsak Pimsarn
===============================================================================
Features:
- Parametric 2D/3D curved beam/frame finite element formulation
- Linear elasticity with local-to-global coordinate transformation
- Boundary conditions: Pinned front eye, Roller/shackle rear eye, Center axle load
- Multiple load cases: Static, 2G Bump, Braking, Cornering
- Strain Energy and Von Mises Stress convergence study
- Model Revision: Uniform Thickness vs. Parabolic Tapered Leaf Spring
- Generates publication-quality charts saved to ../scripts/
===============================================================================
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import os

# Create output folder for figures
scripts_dir = os.path.dirname(os.path.abspath(__file__))
project_dir = os.path.dirname(scripts_dir)
output_dir = os.path.join(project_dir, 'results')
os.makedirs(output_dir, exist_ok=True)

# Material Properties: 55Cr3 (SUP9 / SAE 5160 Spring Steel)
E_MODULUS = 206e9        # Pa (206 GPa)
NU = 0.29
RHO = 7850.0             # kg/m^3
YIELD_STRENGTH = 1100e6  # Pa (1100 MPa)
TENSILE_STRENGTH = 1280e6 # Pa (1280 MPa)
G_MODULUS = E_MODULUS / (2.0 * (1.0 + NU))  # Pa (79.845 GPa Shear Modulus)

def build_curved_leaf_spring_mesh(num_elements=40, span=1.200, camber=0.100, width=0.060, thickness_func=None):
    """
    Discretizes a curved leaf spring between x = -span/2 and x = +span/2
    with camber y(x) = camber * (1 - (2x/span)^2).
    thickness_func(x): returns thickness at longitudinal coordinate x.
    """
    if thickness_func is None:
        thickness_func = lambda x: 0.012  # Uniform 12 mm
        
    x_nodes = np.linspace(-span/2.0, span/2.0, num_elements + 1)
    y_nodes = camber * (1.0 - (2.0 * x_nodes / span)**2)
    nodes = np.column_stack((x_nodes, y_nodes))
    
    elements = []
    element_props = []
    
    for i in range(num_elements):
        n1 = i
        n2 = i + 1
        x_mid = 0.5 * (nodes[n1, 0] + nodes[n2, 0])
        t_el = thickness_func(x_mid)
        b_el = width
        A_el = b_el * t_el
        I_el = (b_el * (t_el**3)) / 12.0
        Z_el = (b_el * (t_el**2)) / 6.0
        
        elements.append((n1, n2))
        element_props.append({
            "length": np.linalg.norm(nodes[n2] - nodes[n1]),
            "A": A_el,
            "I": I_el,
            "Z": Z_el,
            "thickness": t_el,
            "width": b_el,
            "x_mid": x_mid
        })
        
    return nodes, elements, element_props

def solve_leaf_spring_fem(num_elements=40, F_vertical=-8240.4, F_horizontal=0.0, thickness_func=None):
    """
    Solves 2D Frame/Beam FEM for the leaf spring under given loads.
    DOFs per node: [u_x, u_y, theta_z] (3 DOFs per node)
    """
    nodes, elements, element_props = build_curved_leaf_spring_mesh(
        num_elements=num_elements, thickness_func=thickness_func
    )
    num_nodes = len(nodes)
    total_dofs = 3 * num_nodes
    
    K_global = np.zeros((total_dofs, total_dofs))
    
    for idx, (n1, n2) in enumerate(elements):
        prop = element_props[idx]
        L_el = prop["length"]
        A_el = prop["A"]
        I_el = prop["I"]
        
        dx = nodes[n2, 0] - nodes[n1, 0]
        dy = nodes[n2, 1] - nodes[n1, 1]
        c = dx / L_el
        s = dy / L_el
        
        # Local 2D beam stiffness matrix
        k_axial = (E_MODULUS * A_el) / L_el
        k12 = (12.0 * E_MODULUS * I_el) / (L_el**3)
        k6 = (6.0 * E_MODULUS * I_el) / (L_el**2)
        k4 = (4.0 * E_MODULUS * I_el) / L_el
        k2 = (2.0 * E_MODULUS * I_el) / L_el
        
        k_local = np.array([
            [ k_axial,  0,      0,     -k_axial,  0,      0     ],
            [ 0,        k12,    k6,     0,       -k12,    k6    ],
            [ 0,        k6,     k4,     0,       -k6,     k2    ],
            [-k_axial,  0,      0,      k_axial,  0,      0     ],
            [ 0,       -k12,   -k6,     0,        k12,   -k6    ],
            [ 0,        k6,     k2,     0,       -k6,     k4    ]
        ])
        
        # Transformation matrix T (6x6)
        T = np.array([
            [ c,  s, 0,  0,  0, 0],
            [-s,  c, 0,  0,  0, 0],
            [ 0,  0, 1,  0,  0, 0],
            [ 0,  0, 0,  c,  s, 0],
            [ 0,  0, 0, -s,  c, 0],
            [ 0,  0, 0,  0,  0, 1]
        ])
        
        k_element_global = T.T @ k_local @ T
        
        dofs = [3*n1, 3*n1+1, 3*n1+2, 3*n2, 3*n2+1, 3*n2+2]
        for r in range(6):
            for col in range(6):
                K_global[dofs[r], dofs[col]] += k_element_global[r, col]
                
    # ---------------------------------------------------------
    # Boundary Conditions
    # ---------------------------------------------------------
    # Front Eye (Node 0, at x = -span/2): Pinned (Ux = 0, Uy = 0, Rot free)
    # Rear Eye (Node num_nodes - 1, at x = +span/2): Roller/Shackle (Uy = 0, Ux free, Rot free)
    fixed_dofs = [0, 1, 3 * (num_nodes - 1) + 1]
    
    # ---------------------------------------------------------
    # Force Vector
    # ---------------------------------------------------------
    F_ext = np.zeros(total_dofs)
    # Center axle load applied at the midpoint node (num_nodes // 2)
    center_node = num_nodes // 2
    F_ext[3 * center_node + 1] += F_vertical    # Vertical force (-Y)
    F_ext[3 * center_node]     += F_horizontal  # Longitudinal force (+X / -X)
    
    # Solve system: [K_free]{U_free} = {F_free}
    free_dofs = np.setdiff1d(np.arange(total_dofs), fixed_dofs)
    U = np.zeros(total_dofs)
    U[free_dofs] = np.linalg.solve(K_global[np.ix_(free_dofs, free_dofs)], F_ext[free_dofs])
    
    # Total strain energy: U_strain = 0.5 * {U}^T [K] {U}
    total_strain_energy = 0.5 * (U @ K_global @ U)
    
    # ---------------------------------------------------------
    # ANSYS Workbench Solution Recovery
    # ---------------------------------------------------------
    # 1. Total Deformation & 2. Directional Deformation (Y-Axis)
    u_x_nodes_mm = U[0::3] * 1000.0
    u_y_nodes_mm = U[1::3] * 1000.0
    u_total_nodes_mm = np.sqrt(u_x_nodes_mm**2 + u_y_nodes_mm**2)

    element_stresses = []
    element_principal_1 = []
    element_tau_max = []
    element_elastic_strain = []
    element_shear_strain = []
    element_strain_energy = []
    element_u_total = []
    element_u_y = []

    for idx, (n1, n2) in enumerate(elements):
        prop = element_props[idx]
        L_el = prop["length"]
        A_el = prop["A"]
        I_el = prop["I"]
        Z_el = prop["Z"]
        
        dx = nodes[n2, 0] - nodes[n1, 0]
        dy = nodes[n2, 1] - nodes[n1, 1]
        c = dx / L_el
        s = dy / L_el
        
        T = np.array([
            [ c,  s, 0,  0,  0, 0],
            [-s,  c, 0,  0,  0, 0],
            [ 0,  0, 1,  0,  0, 0],
            [ 0,  0, 0,  c,  s, 0],
            [ 0,  0, 0, -s,  c, 0],
            [ 0,  0, 0,  0,  0, 1]
        ])
        
        dofs = [3*n1, 3*n1+1, 3*n1+2, 3*n2, 3*n2+1, 3*n2+2]
        u_elem_local = T @ U[dofs]
        
        # Axial force and bending moments
        N_axial = (E_MODULUS * A_el / L_el) * (u_elem_local[3] - u_elem_local[0])
        M1 = (E_MODULUS * I_el / L_el**2) * (6.0*(u_elem_local[1] - u_elem_local[4]) + L_el*(4.0*u_elem_local[2] + 2.0*u_elem_local[5]))
        M2 = (E_MODULUS * I_el / L_el**2) * (6.0*(u_elem_local[1] - u_elem_local[4]) + L_el*(2.0*u_elem_local[2] + 4.0*u_elem_local[5]))
        M_max = max(abs(M1), abs(M2))
        
        # 4. Equivalent Stress (Von Mises)
        sigma_axial = N_axial / A_el
        sigma_bending = M_max / Z_el
        sigma_von_mises = abs(sigma_axial) + sigma_bending
        
        # 5. Maximum Principal Stress (Peak tensile stress on convex surface)
        sigma_principal_1 = max(0.0, sigma_axial + sigma_bending, -sigma_axial + sigma_bending)
        
        # 6. Maximum Shear Stress (Tresca: tau_max = (sigma_1 - sigma_3)/2)
        tau_max = sigma_principal_1 / 2.0
        
        # 3. Equivalent Elastic Strain (epsilon_eq = sigma_vm / E)
        strain_eq = sigma_von_mises / E_MODULUS
        
        # Maximum Shear Strain (gamma_max = tau_max / G)
        shear_strain = tau_max / G_MODULUS

        # 7. Element Strain Energy (N*m / J)
        e_strain_energy = (L_el / (6.0 * E_MODULUS * I_el)) * (M1**2 + M1 * M2 + M2**2) + (N_axial**2 * L_el) / (2.0 * E_MODULUS * A_el)

        # Deformations at element midpoint
        elem_u_tot = 0.5 * (u_total_nodes_mm[n1] + u_total_nodes_mm[n2])
        elem_u_vert = 0.5 * (u_y_nodes_mm[n1] + u_y_nodes_mm[n2])
        
        element_stresses.append(sigma_von_mises)
        element_principal_1.append(sigma_principal_1)
        element_tau_max.append(tau_max)
        element_elastic_strain.append(strain_eq)
        element_shear_strain.append(shear_strain)
        element_strain_energy.append(e_strain_energy)
        element_u_total.append(elem_u_tot)
        element_u_y.append(elem_u_vert)
        
    element_stresses = np.array(element_stresses)
    element_principal_1 = np.array(element_principal_1)
    element_tau_max = np.array(element_tau_max)
    element_elastic_strain = np.array(element_elastic_strain)
    element_shear_strain = np.array(element_shear_strain)
    element_strain_energy = np.array(element_strain_energy)
    element_u_total = np.array(element_u_total)
    element_u_y = np.array(element_u_y)
    
    # Calculate mass of leaf spring
    total_volume = sum(p["A"] * p["length"] for p in element_props)
    total_mass = total_volume * RHO
    
    return {
        "nodes": nodes,
        "elements": elements,
        "element_props": element_props,
        "U": U,
        "u_x_nodes_mm": u_x_nodes_mm,
        "u_y_nodes_mm": u_y_nodes_mm,
        "u_total_nodes_mm": u_total_nodes_mm,
        "element_u_total_mm": element_u_total,
        "element_u_y_mm": element_u_y,
        "center_deflection_mm": abs(U[3 * center_node + 1]) * 1000.0,
        "max_von_mises_MPa": np.max(element_stresses) / 1e6,
        "element_stresses_MPa": element_stresses / 1e6,
        "max_principal_stress_MPa": np.max(element_principal_1) / 1e6,
        "element_principal_1_MPa": element_principal_1 / 1e6,
        "max_shear_stress_MPa": np.max(element_tau_max) / 1e6,
        "element_tau_max_MPa": element_tau_max / 1e6,
        "max_elastic_strain": float(np.max(element_elastic_strain)),
        "element_strain_eq": element_elastic_strain,
        "max_microstrain": float(np.max(element_elastic_strain)) * 1e6,
        "element_microstrain": element_elastic_strain * 1e6,
        "max_shear_strain": float(np.max(element_shear_strain)),
        "element_shear_strain": element_shear_strain,
        "max_shear_microstrain": float(np.max(element_shear_strain)) * 1e6,
        "element_shear_microstrain": element_shear_strain * 1e6,
        "element_strain_energy_J": element_strain_energy,
        "max_element_strain_energy_J": float(np.max(element_strain_energy)),
        "strain_energy_J": total_strain_energy,
        "total_mass_kg": total_mass,
        "FoS": YIELD_STRENGTH / np.max(element_stresses)
    }

def run_mesh_convergence_study():
    print("\n" + "=" * 70)
    print(" 3. STRAIN ENERGY & MESH CONVERGENCE STUDY")
    print("=" * 70)
    
    element_counts = [10, 20, 40, 80, 160]
    results_conv = []
    
    F_test = -8240.4 # N (Static load)
    
    for n_el in element_counts:
        res = solve_leaf_spring_fem(num_elements=n_el, F_vertical=F_test)
        results_conv.append({
            "elements": n_el,
            "nodes": n_el + 1,
            "max_stress": res["max_von_mises_MPa"],
            "deflection": res["center_deflection_mm"],
            "strain_energy": res["strain_energy_J"]
        })
        print(f"Elements: {n_el:3d} | Nodes: {n_el+1:3d} | Max Stress: {res['max_von_mises_MPa']:6.2f} MPa | "
              f"Deflection: {res['center_deflection_mm']:5.2f} mm | Strain Energy: {res['strain_energy_J']:.4f} N.m")
              
    # Plot Convergence Graph (like Figure 24 of Dr. Monsak's report)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    
    els = [r["elements"] for r in results_conv]
    se = [r["strain_energy"] for r in results_conv]
    st = [r["max_stress"] for r in results_conv]
    
    ax1.plot(els, se, 'ro-', linewidth=2, markersize=7)
    ax1.set_title("Strain Energy Convergence (Leaf Spring)", fontsize=12, fontweight='bold')
    ax1.set_xlabel("Number of Elements", fontsize=11)
    ax1.set_ylabel("Total Strain Energy (N.m)", fontsize=11)
    ax1.grid(True, linestyle='--', alpha=0.6)
    
    ax2.plot(els, st, 'bs-', linewidth=2, markersize=7)
    ax2.axhline(y=YIELD_STRENGTH/1e6, color='r', linestyle='--', label=f"Yield Limit ({YIELD_STRENGTH/1e6:.0f} MPa)")
    ax2.set_title("Max Von Mises Stress vs Mesh Refinement", fontsize=12, fontweight='bold')
    ax2.set_xlabel("Number of Elements", fontsize=11)
    ax2.set_ylabel("Max Von Mises Stress (MPa)", fontsize=11)
    ax2.legend()
    ax2.grid(True, linestyle='--', alpha=0.6)
    
    plt.tight_layout()
    chart_path = os.path.join(output_dir, "mesh_convergence_study.png")
    plt.savefig(chart_path, dpi=300)
    plt.close()
    print(f"\n[SAVED] Mesh convergence chart saved to: {chart_path}")
    
    return results_conv

def plot_ansys_workbench_solutions(res, output_path=None):
    """
    Generates an executive 8-panel dashboard corresponding to the 7 standard
    ANSYS Workbench Mechanical solution fields + Solution Summary Dashboard:
    1. Total Deformation (mm)
    2. Directional Deformation [Y-Axis] (mm)
    3. Equivalent Elastic Strain (microstrain)
    4. Equivalent Stress [Von-Mises] (MPa)
    5. Maximum Principal Stress (MPa)
    6. Maximum Shear Stress (MPa)
    7. Strain Energy (N*m / J)
    8. ANSYS Solution Summary Dashboard
    """
    if output_path is None:
        output_path = os.path.join(output_dir, "ansys_workbench_solutions.png")

    nodes = res["nodes"]
    elements = res["elements"]
    x_orig = nodes[:, 0] * 1000.0
    y_orig = nodes[:, 1] * 1000.0

    x_def = (nodes[:, 0] + res["U"][0::3]) * 1000.0
    y_def = (nodes[:, 1] + res["U"][1::3]) * 1000.0

    points = np.column_stack([x_def, y_def])
    segs = [np.array([points[n1], points[n2]]) for (n1, n2) in elements]

    fig, axes = plt.subplots(2, 4, figsize=(22, 10.5))
    fig.patch.set_facecolor('#F8FAFC')

    solution_configs = [
        {
            "data": res["element_u_total_mm"],
            "title": "1. Total Deformation",
            "unit": "mm",
            "subtitle": "Resultant Displacement | sqrt(Ux^2 + Uy^2)",
            "cmap": "turbo",
            "fmt": "{:.2f}"
        },
        {
            "data": res["element_u_y_mm"],
            "title": "2. Directional Deformation [Y-Axis]",
            "unit": "mm",
            "subtitle": "Vertical Deflection along Gravity / Ride Axis",
            "cmap": "turbo",
            "fmt": "{:.2f}"
        },
        {
            "data": res["element_microstrain"],
            "title": "3. Equivalent Elastic Strain",
            "unit": "μm/m",
            "subtitle": f"Von Mises Strain | ε_eq = σ_vm / E (Yield: {1100e6/E_MODULUS*1e6:.0f} μm/m)",
            "cmap": "turbo",
            "fmt": "{:.1f}"
        },
        {
            "data": res["element_stresses_MPa"],
            "title": "4. Equivalent Stress (Von Mises)",
            "unit": "MPa",
            "subtitle": f"Octahedral Distortion Metric (Yield: {YIELD_STRENGTH/1e6:.0f} MPa)",
            "cmap": "turbo",
            "fmt": "{:.1f}"
        },
        {
            "data": res["element_principal_1_MPa"],
            "title": "5. Maximum Principal Stress (σ₁)",
            "unit": "MPa",
            "subtitle": "Peak Tensile Normal Stress (Fatigue Crack Initiation)",
            "cmap": "turbo",
            "fmt": "{:.1f}"
        },
        {
            "data": res["element_tau_max_MPa"],
            "title": "6. Maximum Shear Stress (Tresca)",
            "unit": "MPa",
            "subtitle": f"Tresca Metric: τ_max = σ₁/2 | Peak γ = {res['max_shear_microstrain']:.1f} μm/m",
            "cmap": "turbo",
            "fmt": "{:.1f}"
        },
        {
            "data": res["element_strain_energy_J"],
            "title": "7. Strain Energy Distribution",
            "unit": "N·m (J)",
            "subtitle": f"Elastic Energy per Element | Total: {res['strain_energy_J']:.2f} N·m",
            "cmap": "turbo",
            "fmt": "{:.3f}"
        }
    ]

    # Plot the first 7 panels
    for idx in range(7):
        ax = axes.flat[idx]
        sc = solution_configs[idx]
        ax.set_facecolor('#FFFFFF')
        data = sc["data"]
        val_min = float(np.min(data))
        val_max = float(np.max(data))

        # Undeformed shape
        ax.plot(x_orig, y_orig, color='#94A3B8', linestyle='--', linewidth=1.2, alpha=0.6, label='Undeformed')

        # Deformed colored beam
        lc = LineCollection(segs, cmap=sc["cmap"], linewidths=7.0)
        lc.set_array(data)
        lc.set_clim(val_min, val_max)
        ax.add_collection(lc)

        # Callouts for Max & Min
        idx_max = np.argmax(data)
        idx_min = np.argmin(data)
        x_max = 0.5 * (x_def[elements[idx_max][0]] + x_def[elements[idx_max][1]])
        y_max = 0.5 * (y_def[elements[idx_max][0]] + y_def[elements[idx_max][1]])
        x_min = 0.5 * (x_def[elements[idx_min][0]] + x_def[elements[idx_min][1]])
        y_min = 0.5 * (y_def[elements[idx_min][0]] + y_def[elements[idx_min][1]])

        ax.scatter([x_max], [y_max], color='#DC2626', s=70, edgecolors='black', zorder=5, marker='^')
        ax.scatter([x_min], [y_min], color='#2563EB', s=70, edgecolors='black', zorder=5, marker='v')

        txt_info = f"Max: {sc['fmt'].format(val_max)} {sc['unit']}\nMin: {sc['fmt'].format(val_min)} {sc['unit']}"
        ax.text(0.03, 0.92, txt_info, transform=ax.transAxes, fontsize=8.5, fontfamily='sans-serif',
                fontweight='bold', verticalalignment='top',
                bbox=dict(boxstyle='round,pad=0.35', facecolor='#F1F5F9', edgecolor='#CBD5E1', alpha=0.9))

        ax.set_title(f"{sc['title']}\n{sc['subtitle']}", fontsize=10, fontweight='bold', pad=10)
        ax.set_xlim(-640, 640)
        ax.set_ylim(-210, 140)
        ax.set_xlabel("X (mm)", fontsize=8)
        ax.set_ylabel("Y (mm)", fontsize=8)
        ax.grid(True, linestyle=':', alpha=0.5)

        cbar = fig.colorbar(lc, ax=ax, fraction=0.032, pad=0.02)
        cbar.set_label(sc["unit"], fontsize=8.5, fontweight='bold')
        cbar.ax.tick_params(labelsize=8)

    # Panel 8: Executive Solution Summary Dashboard
    ax8 = axes.flat[7]
    ax8.set_facecolor('#FFFFFF')
    ax8.axis('off')

    table_data = [
        ["1. Total Deformation", f"{res['center_deflection_mm']:.2f} mm", "Midspan (Axle)"],
        ["2. Directional Def. (Y)", f"-{res['center_deflection_mm']:.2f} mm", "Midspan (Axle)"],
        ["3. Equiv. Elastic Strain", f"{res['max_microstrain']:.1f} um/m", "Clamping Zone"],
        ["4. Equivalent Stress (VM)", f"{res['max_von_mises_MPa']:.2f} MPa", "Clamping Zone"],
        ["5. Max Principal Stress", f"{res['max_principal_stress_MPa']:.2f} MPa", "Tension Surface"],
        ["6. Max Shear Stress", f"{res['max_shear_stress_MPa']:.2f} MPa", "45° Shear Plane"],
        ["7. Total Strain Energy", f"{res['strain_energy_J']:.2f} N·m", "Full Spring Span"]
    ]

    col_labels = ["ANSYS Solution Item", "Max Value", "Critical Zone"]
    tab = ax8.table(cellText=table_data, colLabels=col_labels, loc='center', cellLoc='left',
                    colWidths=[0.42, 0.28, 0.30])
    tab.auto_set_font_size(False)
    tab.set_fontsize(8.5)
    tab.scale(1.0, 1.65)

    # Style header row
    for (row, col), cell in tab.get_celld().items():
        if row == 0:
            cell.set_facecolor('#1E293B')
            cell.set_text_props(color='white', fontweight='bold')
        else:
            if row % 2 == 1:
                cell.set_facecolor('#F8FAFC')
            else:
                cell.set_facecolor('#FFFFFF')
            cell.set_edgecolor('#E2E8F0')

    ax8.set_title("8. ANSYS Solution Summary Dashboard\nFull Metric Breakdown (Static Ride: Fv = -8,240.40 N)",
                  fontsize=10, fontweight='bold', pad=10)

    plt.suptitle("ANSYS Workbench Mechanical Standard - 7 Solution Post-Processing\n"
                 "Automotive Semi-Elliptic Leaf Spring (Static Ride: F_v = -8,240.40 N | Material: 55Cr3 / SUP9 Spring Steel)",
                 fontsize=13, fontweight='bold', y=0.99)
    plt.tight_layout(rect=[0, 0.02, 1, 0.96])
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"\n[SAVED] ANSYS Workbench 7-Solution figure saved to: {output_path}")

def run_load_cases_analysis():
    print("\n" + "=" * 70)
    print(" 4. MULTI-LOAD CASE STRUCTURAL ANALYSIS (SOLID SPRING STEEL)")
    print("=" * 70)
    
    cases = [
        ("Case 1: Normal Static Vertical Load", -8240.4, 0.0),
        ("Case 2: Peak Dynamic Bump Load (2.0G)", -16480.8, 0.0),
        ("Case 3: Combined Bump + Braking Load", -8240.4, -3500.0),
        ("Case 4: Severe Dynamic Bump + Braking", -16480.8, -5000.0),
    ]
    
    study_results = []
    
    for idx_c, (case_name, F_v, F_h) in enumerate(cases):
        res = solve_leaf_spring_fem(num_elements=60, F_vertical=F_v, F_horizontal=F_h)
        status = "SAFE" if res["FoS"] >= 1.0 else "EXCEEDS YIELD"
        study_results.append({
            "case": case_name,
            "F_v": F_v,
            "F_h": F_h,
            "max_stress": res["max_von_mises_MPa"],
            "deflection": res["center_deflection_mm"],
            "FoS": res["FoS"],
            "status": status,
            "res": res
        })
        print(f"\n[{case_name}]")
        print(f"  1. Total Deformation:           {res['center_deflection_mm']:8.2f} mm")
        print(f"  2. Directional Def. (Y-Axis):  -{res['center_deflection_mm']:8.2f} mm")
        print(f"  3. Equivalent Elastic Strain:  {res['max_microstrain']:8.1f} um/m (microstrain)")
        print(f"  4. Equivalent Stress (Von Mises): {res['max_von_mises_MPa']:6.2f} MPa (Yield: {YIELD_STRENGTH/1e6:.0f} MPa)")
        print(f"  5. Maximum Principal Stress (sigma_1): {res['max_principal_stress_MPa']:6.2f} MPa")
        print(f"  6. Maximum Shear Stress (tau_max):     {res['max_shear_stress_MPa']:6.2f} MPa")
        print(f"  7. Total Strain Energy:         {res['strain_energy_J']:8.2f} N.m")
        print(f"  >> Factor of Safety (FoS):       {res['FoS']:8.2f} -> {status}")

        if idx_c == 0:
            # Generate ANSYS Workbench 7-solution publication figure for Case 1
            plot_ansys_workbench_solutions(res)

    return study_results

def run_model_revision():
    print("\n" + "=" * 70)
    print(" 5. MODEL REVISION: UNIFORM THICKNESS vs. PARABOLIC TAPERED LEAF SPRING")
    print("=" * 70)
    
    span = 1.200
    # Parabolic tapering function:
    # At center (x=0): t = 14.5 mm
    # At eyes (x = +/- span/2): t = 7.5 mm
    def parabolic_taper(x):
        ratio = abs(2.0 * x / span)
        return 0.0145 - (0.0145 - 0.0075) * (ratio**2)
        
    F_bump = -16480.8 # 2G bump
    
    res_baseline = solve_leaf_spring_fem(num_elements=60, F_vertical=F_bump, thickness_func=None)
    res_revised = solve_leaf_spring_fem(num_elements=60, F_vertical=F_bump, thickness_func=parabolic_taper)
    
    print(f"BASELINE MODEL (Uniform 12 mm):")
    print(f"  Mass: {res_baseline['total_mass_kg']:.2f} kg | Max Stress: {res_baseline['max_von_mises_MPa']:.2f} MPa | Deflection: {res_baseline['center_deflection_mm']:.2f} mm | FoS: {res_baseline['FoS']:.2f}")
    
    print(f"\nREVISED MODEL (Parabolic Tapered 14.5 -> 7.5 mm):")
    print(f"  Mass: {res_revised['total_mass_kg']:.2f} kg | Max Stress: {res_revised['max_von_mises_MPa']:.2f} MPa | Deflection: {res_revised['center_deflection_mm']:.2f} mm | FoS: {res_revised['FoS']:.2f}")
    
    mass_saving = (res_baseline['total_mass_kg'] - res_revised['total_mass_kg']) / res_baseline['total_mass_kg'] * 100.0
    stress_reduction = (res_baseline['max_von_mises_MPa'] - res_revised['max_von_mises_MPa']) / res_baseline['max_von_mises_MPa'] * 100.0
    
    print(f"\n>>> REVISION SUMMARY:")
    print(f"  Mass Reduction: {mass_saving:.1f}%")
    print(f"  Stress Reduction: {stress_reduction:.1f}%")
    print(f"  FoS Improvement: {res_baseline['FoS']:.2f} -> {res_revised['FoS']:.2f}")
    
    # Plot Comparison Chart
    x_coords = [p["x_mid"] * 1000.0 for p in res_baseline["element_props"]]
    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.plot(x_coords, res_baseline["element_stresses_MPa"], 'r--', linewidth=2, label="Baseline (Uniform 12 mm)")
    ax.plot(x_coords, res_revised["element_stresses_MPa"], 'b-', linewidth=2, label="Revised (Parabolic Tapered)")
    ax.axhline(y=YIELD_STRENGTH/1e6, color='k', linestyle=':', label="Yield Strength (1100 MPa)")
    ax.set_title("Stress Distribution Along Leaf Spring (2.0G Bump Load)", fontsize=12, fontweight='bold')
    ax.set_xlabel("Longitudinal Position from Center (mm)", fontsize=11)
    ax.set_ylabel("Von Mises Stress (MPa)", fontsize=11)
    ax.legend()
    ax.grid(True, linestyle='--', alpha=0.6)
    plt.tight_layout()
    chart_path = os.path.join(output_dir, "model_revision_comparison.png")
    plt.savefig(chart_path, dpi=300)
    plt.close()
    print(f"[SAVED] Comparison chart saved to: {chart_path}")

if __name__ == "__main__":
    run_mesh_convergence_study()
    run_load_cases_analysis()
    run_model_revision()
