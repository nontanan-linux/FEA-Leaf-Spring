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
import os

# Create output folder for figures
output_dir = os.path.dirname(os.path.abspath(__file__))

# Material Properties: 55Cr3 (SUP9 / SAE 5160 Spring Steel)
E_MODULUS = 206e9        # Pa (206 GPa)
NU = 0.29
RHO = 7850.0             # kg/m^3
YIELD_STRENGTH = 1100e6  # Pa (1100 MPa)
TENSILE_STRENGTH = 1280e6 # Pa (1280 MPa)

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
    
    # Stress recovery per element
    element_stresses = []
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
        
        sigma_axial = N_axial / A_el
        sigma_bending = M_max / Z_el
        sigma_von_mises = abs(sigma_axial) + sigma_bending
        element_stresses.append(sigma_von_mises)
        
    element_stresses = np.array(element_stresses)
    
    # Calculate mass of leaf spring
    total_volume = sum(p["A"] * p["length"] for p in element_props)
    total_mass = total_volume * RHO
    
    return {
        "nodes": nodes,
        "elements": elements,
        "element_props": element_props,
        "U": U,
        "center_deflection_mm": abs(U[3 * center_node + 1]) * 1000.0,
        "max_von_mises_MPa": np.max(element_stresses) / 1e6,
        "element_stresses_MPa": element_stresses / 1e6,
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
    
    for case_name, F_v, F_h in cases:
        res = solve_leaf_spring_fem(num_elements=60, F_vertical=F_v, F_horizontal=F_h)
        status = "SAFE" if res["FoS"] >= 1.0 else "EXCEEDS YIELD"
        study_results.append({
            "case": case_name,
            "F_v": F_v,
            "F_h": F_h,
            "max_stress": res["max_von_mises_MPa"],
            "deflection": res["center_deflection_mm"],
            "FoS": res["FoS"],
            "status": status
        })
        print(f"\n[{case_name}]")
        print(f"  Max Von Mises Stress: {res['max_von_mises_MPa']:.2f} MPa (Yield: {YIELD_STRENGTH/1e6:.0f} MPa)")
        print(f"  Center Deflection: {res['center_deflection_mm']:.2f} mm")
        print(f"  Factor of Safety (FoS): {res['FoS']:.2f} -> {status}")

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
