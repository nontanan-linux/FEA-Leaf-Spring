"""
===============================================================================
Module 2: Analytical Formulations & Element Equations for Leaf Spring FEM
Course: Finite Element Method in Engineering (M.Eng)
Instructor: Asst. Prof. Dr. Monsak Pimsarn
===============================================================================
This module provides the Object-Oriented class LeafSpringAnalytical:
1. Dynamic load derivations from vehicle specifications in config/vehicle_information.yaml
2. Closed-form semi-elliptic beam mechanics based on config/model.yaml & material.yaml
3. Local 2D beam element stiffness equations [k^e] (6x6)
4. Local-to-global direction cosine coordinate transformation [T] (6x6)
5. Global stiffness matrix assembly [K_global] = sum [T]^T [k^e] [T]
6. External load vector {F_ext} creation and boundary condition degree-of-freedom mapping
===============================================================================
"""

import os
import sys
import numpy as np

# Ensure scripts directory is in python path
scripts_dir = os.path.dirname(os.path.abspath(__file__))
if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)

from config_loader import ConfigManager, config_manager


class LeafSpringAnalytical:
    """
    Encapsulates vehicle dynamics load derivation, analytical beam mechanics,
    element stiffness formulation, and global assembly for automotive leaf springs.
    """

    def __init__(self, config=None, config_dir=None):
        if config is None:
            self.cfg_mgr = ConfigManager(config_dir=config_dir)
        else:
            self.cfg_mgr = config

        # Material Properties (55Cr3 / SUP9 Spring Steel)
        mat = self.cfg_mgr.get_material_properties()
        self.material_name = mat["name"]
        self.E_modulus = mat["E_modulus"]
        self.nu = mat["nu"]
        self.rho = mat["rho"]
        self.yield_strength = mat["yield_strength"]
        self.tensile_strength = mat["tensile_strength"]
        self.G_modulus = mat["G_modulus"]

        # Leaf Spring Dimensions (m)
        dims = self.cfg_mgr.get_leaf_spring_dimensions()
        self.span = dims["span"]
        self.semi_span = dims["span"] / 2.0
        self.camber = dims["camber"]
        self.width = dims["width"]
        self.thickness = dims["thickness"]

        # Vehicle Specifications
        veh = self.cfg_mgr.get_vehicle_properties()
        self.m_vehicle = veh["gvw_kg"]
        self.g = veh["gravity"]
        self.total_weight_n = veh["total_weight_n"]
        self.wheelbase = veh["wheelbase_m"]
        self.track_width = veh["track_width_m"]
        self.h_cg = veh["cg_height_m"]
        self.rear_ratio = veh["rear_axle_ratio"]
        self.front_ratio = 1.0 - self.rear_ratio
        self.mu_tire = 1.0

    # -------------------------------------------------------------------------
    # 1. Vehicle Dynamic Load Transfer Calculations
    # -------------------------------------------------------------------------
    def calculate_vehicle_loads(self, verbose=True):
        """
        Derives operational vehicle suspension forces based on vehicle dynamics principles:
          - A. Static 1G normal ride load
          - B. Peak 2.0G dynamic road bump impact
          - C. Longitudinal 0.8G emergency braking weight transfer
          - D. Lateral 0.7G steady-state cornering weight transfer
        """
        F_g = self.m_vehicle * self.g

        if verbose:
            print("=" * 70)
            print(" 1. VEHICLE SPECIFICATIONS & MASS PROPERTIES (FROM CONFIG)")
            print("=" * 70)
            print(f"Gross Vehicle Weight (GVW): {self.m_vehicle:.1f} kg ({F_g:.2f} N)")
            print(f"Wheelbase: {self.wheelbase:.3f} m | Track Width: {self.track_width:.3f} m | CG Height: {self.h_cg:.3f} m")

        # A. Static Normal Forces (60% rear axle load distribution)
        F_z_rear_total = F_g * self.rear_ratio
        F_z_rear_static_per_spring = F_z_rear_total / 2.0  # 2 rear leaf springs

        if verbose:
            print("\n--- A. STATICS NORMAL FORCES ---")
            print(f"Total Rear Axle Static Normal Force: {F_z_rear_total:.2f} N ({self.rear_ratio*100:.1f}%)")
            print(f"Static Load per Leaf Spring (F_v_static): {F_z_rear_static_per_spring:.2f} N")

        # B. Dynamic Bump Load (2.0G Road Impact)
        K_bump = 2.0
        F_v_bump = F_z_rear_static_per_spring * K_bump
        if verbose:
            print("\n--- B. PEAK DYNAMIC BUMP LOAD (2.0G Impact) ---")
            print(f"Dynamic Factor: {K_bump}G")
            print(f"Design Bump Force per Leaf Spring (F_v_bump): {F_v_bump:.2f} N")

        # C. Longitudinal Weight Transfer During Braking (0.8G Deceleration)
        a_brake = 0.8 * self.g
        delta_W_brake = (self.m_vehicle * a_brake * self.h_cg) / self.wheelbase
        F_z_rear_braking = max(0.0, F_z_rear_total - delta_W_brake)
        F_x_brake_rear_total = F_z_rear_braking * self.mu_tire
        F_x_brake_per_spring = F_x_brake_rear_total / 2.0

        if verbose:
            print("\n--- C. LONGITUDINAL BRAKING LOAD TRANSFER (0.8G) ---")
            print(f"Rear Normal Force under Braking: {F_z_rear_braking:.2f} N")
            print(f"Braking Force per Leaf Spring (F_brake): {F_x_brake_per_spring:.2f} N")

        # D. Lateral Weight Transfer During Cornering (0.7G Lateral Accel)
        a_lat = 0.7 * self.g
        delta_W_lat_rear = (F_z_rear_total * a_lat * self.h_cg) / (self.g * self.track_width)
        F_z_outer_spring = (F_z_rear_total / 2.0) + delta_W_lat_rear
        F_y_corner_per_spring = F_z_outer_spring * self.mu_tire

        if verbose:
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

    # -------------------------------------------------------------------------
    # 2. Analytical Beam Mechanics
    # -------------------------------------------------------------------------
    def calculate_leaf_spring_mechanics(self, forces=None, verbose=True):
        """
        Calculates closed-form semi-elliptic leaf spring mechanics:
          - Bending Moment: M_max = (F / 2) * L_semi
          - Bending Stress: sigma_max = M_max / Z
          - Center Deflection: delta = F * L_semi^3 / (6 * E * I)
          - Spring Stiffness: k = F / delta
          - Factor of Safety: FoS = sigma_yield / sigma_max
        """
        if forces is None:
            forces = self.calculate_vehicle_loads(verbose=False)

        b = self.width
        t = self.thickness
        L_semi = self.semi_span
        E = self.E_modulus
        sigma_yield = self.yield_strength

        I = (b * t**3) / 12.0
        Z_modulus = (b * t**2) / 6.0

        if verbose:
            print("\n" + "=" * 70)
            print(" 2. ANALYTICAL BEAM MECHANICS (SEMI-ELLIPTIC LEAF SPRING)")
            print("=" * 70)
            print(f"Spring Span: {self.span*1000:.0f} mm | Semi-span (L): {L_semi*1000:.0f} mm")
            print(f"Cross Section: Width b = {b*1000:.1f} mm, Thickness t = {t*1000:.1f} mm")
            print(f"Second Moment of Area (I): {I:.4e} m^4 | Section Modulus (Z): {Z_modulus:.4e} m^3")
            print(f"Yield Strength (sigma_y): {sigma_yield/1e6:.1f} MPa")

        cases = [
            ("Case 1: Normal Static Vertical Load", forces["F_v_static"]),
            ("Case 2: Peak Dynamic Bump Load (2.0G)", forces["F_v_bump"]),
        ]

        results = {}
        for case_name, F_applied in cases:
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

            if verbose:
                print(f"\n[{case_name}]")
                print(f"  Applied Center Force (F): {F_applied:.2f} N")
                print(f"  Max Bending Moment (M_max): {M_max:.2f} N.m")
                print(f"  Max Bending Stress (sigma_max): {sigma_max/1e6:.2f} MPa")
                print(f"  Max Center Deflection (delta): {delta_max*1000.0:.2f} mm")
                print(f"  Spring Stiffness (k): {spring_rate/1000.0:.2f} N/mm")
                print(f"  Factor of Safety (FoS): {FoS:.2f} {'[SAFE]' if FoS >= 1.0 else '[WARNING: YIELDING]'}")

        return results

    # -------------------------------------------------------------------------
    # 3. Finite Element Formulations & Assembly
    # -------------------------------------------------------------------------
    def compute_element_stiffness(self, prop, E_modulus=None):
        """
        Computes local 2D Euler-Bernoulli frame element stiffness matrix [k_local] (6x6).
        DOFs per node: [u_x, u_y, theta_z] (3 DOFs per node, 6 DOFs per element).
        """
        if E_modulus is None:
            E_modulus = self.E_modulus

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

    @staticmethod
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

    def assemble_global_stiffness(self, nodes, elements, element_props, E_modulus=None):
        """
        Assembles the global structural stiffness matrix [K_global] from element equations:
        [K_global] = sum_e [T_e]^T [k_e] [T_e]
        """
        if E_modulus is None:
            E_modulus = self.E_modulus

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

            k_local = self.compute_element_stiffness(prop, E_modulus=E_modulus)
            T = self.compute_transformation_matrix(c, s)
            k_global_element = T.T @ k_local @ T

            dofs = [3*n1, 3*n1+1, 3*n1+2, 3*n2, 3*n2+1, 3*n2+2]
            for r in range(6):
                for col in range(6):
                    K_global[dofs[r], dofs[col]] += k_global_element[r, col]

        return K_global

    def create_global_force_vector(self, total_dofs, center_node, F_vertical=-8240.40, F_horizontal=0.0):
        """
        Constructs global external load vector {F_ext}.
        Applies center axle vertical and longitudinal braking forces at center_node.
        """
        F_ext = np.zeros(total_dofs)
        F_ext[3 * center_node + 1] += F_vertical    # Vertical force (Uy, N)
        F_ext[3 * center_node]     += F_horizontal  # Longitudinal force (Ux, N)
        return F_ext

    @staticmethod
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


# -----------------------------------------------------------------------------
# Module-Level Backward-Compatible API & Singletons
# -----------------------------------------------------------------------------
_default_analytical = LeafSpringAnalytical()

# Constants derived from configuration
E_MODULUS = _default_analytical.E_modulus
NU = _default_analytical.nu
RHO = _default_analytical.rho
YIELD_STRENGTH = _default_analytical.yield_strength
TENSILE_STRENGTH = _default_analytical.tensile_strength
G_MODULUS = _default_analytical.G_modulus


def calculate_vehicle_loads():
    return _default_analytical.calculate_vehicle_loads(verbose=True)


def calculate_leaf_spring_mechanics(forces):
    return _default_analytical.calculate_leaf_spring_mechanics(forces, verbose=True)


def compute_element_stiffness(prop, E_modulus=E_MODULUS):
    return _default_analytical.compute_element_stiffness(prop, E_modulus=E_modulus)


def compute_transformation_matrix(c, s):
    return LeafSpringAnalytical.compute_transformation_matrix(c, s)


def assemble_global_stiffness(nodes, elements, element_props, E_modulus=E_MODULUS):
    return _default_analytical.assemble_global_stiffness(nodes, elements, element_props, E_modulus=E_modulus)


def create_global_force_vector(total_dofs, center_node, F_vertical=-8240.40, F_horizontal=0.0):
    return _default_analytical.create_global_force_vector(total_dofs, center_node, F_vertical, F_horizontal)


def identify_boundary_dofs(num_nodes):
    return LeafSpringAnalytical.identify_boundary_dofs(num_nodes)


if __name__ == "__main__":
    print("=" * 70)
    print(" 2. LEAF SPRING ANALYTICAL CLASS & MODULE VERIFICATION")
    print("=" * 70)
    analyzer = LeafSpringAnalytical()
    forces = analyzer.calculate_vehicle_loads()
    results = analyzer.calculate_leaf_spring_mechanics(forces)

    # Unit demonstration of element formulation & assembly via class methods
    print("\n" + "=" * 70)
    print(" 3. TEST FINITE ELEMENT EQUATION & ASSEMBLY (Unit Demo)")
    print("=" * 70)
    from generate_mesh import build_curved_leaf_spring_mesh
    nodes, elements, props = build_curved_leaf_spring_mesh(num_elements=10)
    K = analyzer.assemble_global_stiffness(nodes, elements, props)
    fixed_dofs, free_dofs = analyzer.identify_boundary_dofs(len(nodes))
    F_ext = analyzer.create_global_force_vector(3 * len(nodes), len(nodes) // 2, forces["F_v_static"])
    print(f"Global Stiffness Matrix assembled: size {K.shape[0]}x{K.shape[1]}")
    print(f"Total DOFs: {3*len(nodes)} | Fixed DOFs: {fixed_dofs} | Free DOFs: {len(free_dofs)}")
    print(f"Applied Load: Fv = {forces['F_v_static']:.2f} N at Node {len(nodes)//2}")
    print("[OK] LeafSpringAnalytical class and module execution verified.")
