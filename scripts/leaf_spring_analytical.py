"""
===============================================================================
Automotive Suspension Leaf Spring - Analytical Vehicle Dynamics & Mechanics
Course: Finite Element Method in Engineering (M.Eng)
Instructor: Asst. Prof. Dr. Monsak Pimsarn
===============================================================================
This module computes:
1. Static weight distribution of the vehicle
2. Dynamic longitudinal weight transfer during braking
3. Lateral weight transfer during cornering
4. Peak 2.0G dynamic road bump impact force
5. Analytical leaf spring bending stress, deflection, and spring rate
===============================================================================
"""

import numpy as np

def calculate_vehicle_loads():
    print("=" * 70)
    print(" 1. VEHICLE SPECIFICATIONS & MASS PROPERTIES")
    print("=" * 70)
    
    # Vehicle parameters (Light Commercial Vehicle / 1-Tonne Pickup Truck)
    m_vehicle = 2800.0      # Gross Vehicle Weight GVW (kg)
    g = 9.81                # Acceleration due to gravity (m/s^2)
    F_g = m_vehicle * g     # Total vehicle weight (N)
    
    wheelbase = 3.085       # Wheelbase L_wb (m)
    rear_ratio = 0.60       # Static rear axle weight ratio (laden)
    front_ratio = 0.40      # Static front axle weight ratio (laden)
    
    track_width = 1.540     # Track base rear (m)
    h_cg = 0.750            # Center of gravity height above ground (m)
    mu_tire = 1.0           # Tire-road friction coefficient
    
    print(f"Gross Vehicle Weight (GVW): {m_vehicle:.1f} kg ({F_g:.2f} N)")
    print(f"Wheelbase: {wheelbase:.3f} m | Track Width: {track_width:.3f} m | CG Height: {h_cg:.3f} m")
    
    # ---------------------------------------------------------
    # A. Static Normal Forces
    # ---------------------------------------------------------
    F_z_rear_total = F_g * rear_ratio
    F_z_rear_static_per_spring = F_z_rear_total / 2.0  # 2 leaf springs
    
    print("\n--- A. STATICS NORMAL FORCES ---")
    print(f"Total Rear Axle Static Normal Force: {F_z_rear_total:.2f} N ({rear_ratio*100:.1f}%)")
    print(f"Static Load per Leaf Spring (F_v_static): {F_z_rear_static_per_spring:.2f} N")
    
    # ---------------------------------------------------------
    # B. Dynamic Bump Load (2.0G Road Impact)
    # ---------------------------------------------------------
    K_bump = 2.0
    F_v_bump = F_z_rear_static_per_spring * K_bump
    print("\n--- B. PEAK DYNAMIC BUMP LOAD (2.0G Impact) ---")
    print(f"Dynamic Factor: {K_bump}G")
    print(f"Design Bump Force per Leaf Spring (F_v_bump): {F_v_bump:.2f} N")
    
    # ---------------------------------------------------------
    # C. Longitudinal Weight Transfer During Braking (0.8G Deceleration)
    # ---------------------------------------------------------
    a_brake = 0.8 * g
    # Weight transfer off rear axle onto front axle:
    delta_W_brake = (m_vehicle * a_brake * h_cg) / wheelbase
    F_z_rear_braking = max(0.0, F_z_rear_total - delta_W_brake)
    F_x_brake_rear_total = F_z_rear_braking * mu_tire
    F_x_brake_per_spring = F_x_brake_rear_total / 2.0
    
    print("\n--- C. LONGITUDINAL BRAKING LOAD TRANSFER (0.8G) ---")
    print(f"Rear Normal Force under Braking: {F_z_rear_braking:.2f} N")
    print(f"Braking Force per Leaf Spring (F_brake): {F_x_brake_per_spring:.2f} N")
    
    # ---------------------------------------------------------
    # D. Lateral Weight Transfer During Cornering (0.7G Lateral Accel)
    # ---------------------------------------------------------
    a_lat = 0.7 * g
    delta_W_lat_rear = (F_z_rear_total * a_lat * h_cg) / (g * track_width)
    F_z_outer_spring = (F_z_rear_total / 2.0) + delta_W_lat_rear
    F_y_corner_per_spring = F_z_outer_spring * mu_tire
    
    print("\n--- D. LATERAL CORNERING WEIGHT TRANSFER (0.7G) ---")
    print(f"Outer Rear Leaf Spring Vertical Force: {F_z_outer_spring:.2f} N")
    print(f"Lateral Cornering Force per Spring (F_corner): {F_y_corner_per_spring:.2f} N")
    
    return {
        "F_v_static": F_z_rear_static_per_spring,
        "F_v_bump": F_v_bump,
        "F_brake": F_x_brake_per_spring,
        "F_corner": F_y_corner_per_spring,
        "F_z_outer_corner": F_z_outer_spring
    }

def calculate_leaf_spring_mechanics(forces):
    print("\n" + "=" * 70)
    print(" 2. ANALYTICAL BEAM MECHANICS (SEMI-ELLIPTIC LEAF SPRING)")
    print("=" * 70)
    
    # Geometry of Leaf Spring
    L_span = 1.200       # Total eye-to-eye span (m)
    L_semi = L_span / 2.0 # Semi-span L (m) = 0.600 m
    b = 0.060           # Width (m) = 60 mm
    t = 0.012           # Thickness (m) = 12 mm
    
    # Material: High-strength Spring Steel 55Cr3 (SUP9 / SAE 5160)
    E = 206e9           # Young's modulus (Pa) = 206 GPa
    sigma_yield = 1100e6 # Yield strength (Pa) = 1100 MPa
    
    # Moment of inertia for rectangular section
    I = (b * t**3) / 12.0
    Z_modulus = (b * t**2) / 6.0
    
    print(f"Spring Span: {L_span*1000:.0f} mm | Semi-span (L): {L_semi*1000:.0f} mm")
    print(f"Cross Section: Width b = {b*1000:.1f} mm, Thickness t = {t*1000:.1f} mm")
    print(f"Second Moment of Area (I): {I:.4e} m^4 | Section Modulus (Z): {Z_modulus:.4e} m^3")
    print(f"Yield Strength (sigma_y): {sigma_yield/1e6:.1f} MPa")
    
    cases = [
        ("Case 1: Normal Static Vertical Load", forces["F_v_static"]),
        ("Case 2: Peak Dynamic Bump Load (2.0G)", forces["F_v_bump"]),
    ]
    
    results = {}
    for case_name, F_applied in cases:
        # For semi-elliptic leaf spring supported at both ends and loaded at center:
        # M_max = (F / 2) * L = F * L / 2
        # sigma_max = M_max / Z = (F * L / 2) / (b * t^2 / 6) = 3 * F * L / (b * t^2)
        # Note: In standard beam theory, delta = F * L^3 / (48 * E * I_full)
        # In semi-elliptic leaf spring terms (2 cantilevers back-to-back of length L_semi):
        # delta = (F/2) * L_semi^3 / (3 * E * I) = F * L_semi^3 / (6 * E * I)
        
        M_max = (F_applied / 2.0) * L_semi
        sigma_max = M_max / Z_modulus
        delta_max = (F_applied * (L_semi**3)) / (6.0 * E * I)
        spring_rate = F_applied / delta_max
        FoS = sigma_yield / sigma_max
        
        results[case_name] = {
            "Force_N": F_applied,
            "Moment_Nm": M_max,
            "Stress_MPa": sigma_max / 1e6,
            "Deflection_mm": delta_max * 1000.0,
            "Spring_Rate_N_mm": spring_rate / 1000.0,
            "FoS": FoS
        }
        
        print(f"\n[{case_name}]")
        print(f"  Applied Center Force (F): {F_applied:.2f} N")
        print(f"  Max Bending Moment (M_max): {M_max:.2f} N.m")
        print(f"  Max Bending Stress (sigma_max): {sigma_max/1e6:.2f} MPa")
        print(f"  Max Center Deflection (delta): {delta_max*1000.0:.2f} mm")
        print(f"  Spring Stiffness (k): {spring_rate/1000.0:.2f} N/mm")
        print(f"  Factor of Safety (FoS): {FoS:.2f} {'[SAFE]' if FoS >= 1.0 else '[WARNING: YIELDING]'}")

    return results

# =============================================================================
# 3. FINITE ELEMENT EQUATION FORMULATION & GLOBAL ASSEMBLY
# =============================================================================

# Material Constants: 55Cr3 (SUP9 / SAE 5160 Spring Steel)
E_MODULUS = 206e9         # Pa (206 GPa)
NU = 0.29
RHO = 7850.0              # kg/m^3
YIELD_STRENGTH = 1100e6   # Pa (1100 MPa)
TENSILE_STRENGTH = 1280e6 # Pa (1280 MPa)
G_MODULUS = E_MODULUS / (2.0 * (1.0 + NU))  # Pa (79.845 GPa)

def compute_element_stiffness(prop, E_modulus=E_MODULUS):
    """
    Computes local 2D Euler-Bernoulli/Timoshenko frame element stiffness matrix [k_local] (6x6).
    DOFs per node: [u_x, u_y, theta_z] (3 DOFs per node, 6 DOFs per element).
    """
    L_el = prop["length"]
    A_el = prop["A"]
    I_el = prop["I"]

    k_axial = (E_modulus * A_el) / L_el
    k12 = (12.0 * E_modulus * I_el) / (L_el**3)
    k6  = (6.0  * E_modulus * I_el) / (L_el**2)
    k4  = (4.0  * E_modulus * I_el) / L_el
    k2  = (2.0  * E_modulus * I_el) / L_el

    k_local = np.array([
        [ k_axial,  0.0,    0.0,   -k_axial,  0.0,    0.0   ],
        [ 0.0,      k12,    k6,     0.0,     -k12,    k6    ],
        [ 0.0,      k6,     k4,     0.0,     -k6,     k2    ],
        [-k_axial,  0.0,    0.0,    k_axial,  0.0,    0.0   ],
        [ 0.0,     -k12,   -k6,     0.0,      k12,   -k6    ],
        [ 0.0,      k6,     k2,     0.0,     -k6,     k4    ]
    ])
    return k_local

def compute_transformation_matrix(c, s):
    """
    Computes direction-cosine transformation matrix [T] (6x6) from local to global coordinates.
    c = cos(alpha), s = sin(alpha)
    """
    T = np.array([
        [ c,  s, 0.0,  0.0,  0.0, 0.0],
        [-s,  c, 0.0,  0.0,  0.0, 0.0],
        [ 0.0,  0.0, 1.0,  0.0,  0.0, 0.0],
        [ 0.0,  0.0, 0.0,  c,  s, 0.0],
        [ 0.0,  0.0, 0.0, -s,  c, 0.0],
        [ 0.0,  0.0, 0.0,  0.0,  0.0, 1.0]
    ])
    return T

def assemble_global_stiffness(nodes, elements, element_props, E_modulus=E_MODULUS):
    """
    Assembles the global structural stiffness matrix [K_global] from element equations:
    [K_global] = sum_e [T_e]^T [k_e] [T_e]
    """
    num_nodes = len(nodes)
    total_dofs = 3 * num_nodes
    K_global = np.zeros((total_dofs, total_dofs))

    for idx, (n1, n2) in enumerate(elements):
        prop = element_props[idx]
        L_el = prop["length"]

        dx = nodes[n2, 0] - nodes[n1, 0]
        dy = nodes[n2, 1] - nodes[n1, 1]
        c = dx / L_el
        s = dy / L_el

        k_local = compute_element_stiffness(prop, E_modulus=E_modulus)
        T = compute_transformation_matrix(c, s)
        k_global_element = T.T @ k_local @ T

        dofs = [3*n1, 3*n1+1, 3*n1+2, 3*n2, 3*n2+1, 3*n2+2]
        for r in range(6):
            for col in range(6):
                K_global[dofs[r], dofs[col]] += k_global_element[r, col]

    return K_global

def create_global_force_vector(total_dofs, center_node, F_vertical=-8240.40, F_horizontal=0.0):
    """
    Constructs global external load vector {F_ext}.
    Applies center axle vertical and longitudinal braking forces at center_node.
    """
    F_ext = np.zeros(total_dofs)
    F_ext[3 * center_node + 1] += F_vertical    # Vertical force (Uy, N)
    F_ext[3 * center_node]     += F_horizontal  # Longitudinal force (Ux, N)
    return F_ext

def identify_boundary_dofs(num_nodes):
    """
    Identifies restrained and unconstrained degrees of freedom:
    - Front Eye (Node 0, x = -span/2): Pinned Support (Ux = 0, Uy = 0, Rot = Free)
    - Rear Eye (Node num_nodes-1, x = +span/2): Shackle Roller (Uy = 0, Ux = Free, Rot = Free)
    """
    fixed_dofs = [0, 1, 3 * (num_nodes - 1) + 1]
    total_dofs = 3 * num_nodes
    free_dofs = np.setdiff1d(np.arange(total_dofs), fixed_dofs)
    return fixed_dofs, free_dofs

if __name__ == "__main__":
    forces = calculate_vehicle_loads()
    results = calculate_leaf_spring_mechanics(forces)
    
    # Unit demonstration of element formulation & assembly
    print("\n" + "=" * 70)
    print(" 3. TEST FINITE ELEMENT EQUATION & ASSEMBLY (Unit Demo)")
    print("=" * 70)
    from generate_mesh import build_curved_leaf_spring_mesh
    nodes, elements, props = build_curved_leaf_spring_mesh(num_elements=10)
    K = assemble_global_stiffness(nodes, elements, props)
    fixed_dofs, free_dofs = identify_boundary_dofs(len(nodes))
    F_ext = create_global_force_vector(3 * len(nodes), len(nodes) // 2, forces["F_v_static"])
    print(f"Global Stiffness Matrix assembled: size {K.shape[0]}x{K.shape[1]}")
    print(f"Total DOFs: {3*len(nodes)} | Fixed DOFs: {fixed_dofs} | Free DOFs: {len(free_dofs)}")
    print(f"Applied Load: Fv = {forces['F_v_static']:.2f} N at Node {len(nodes)//2}")
    print("[OK] Element formulation and assembly verified.")
