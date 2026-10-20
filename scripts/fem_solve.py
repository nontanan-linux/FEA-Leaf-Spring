"""
===============================================================================
Module 3: Finite Element System Solver for Automotive Leaf Spring
Course: Finite Element Method in Engineering (M.Eng)
Instructor: Asst. Prof. Dr. Monsak Pimsarn
===============================================================================
This module provides the Object-Oriented class LeafSpringFEMSolver:
1. Imposition of boundary conditions (Partitioning / Reduced System Method)
2. Direct linear equation solution: [K_free] {U_free} = {F_free}
3. Reconstruction of global nodal displacement vector {U}
4. Support reaction forces computation: {R} = [K] {U} - {F_ext}
5. Total elastic strain energy evaluation: U_strain = 0.5 * {U}^T [K] {U}
===============================================================================
"""

import os
import sys
import numpy as np

# Ensure scripts directory is in python path
scripts_dir = os.path.dirname(os.path.abspath(__file__))
if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)

from config_loader import ConfigManager
from generate_mesh import build_curved_leaf_spring_mesh
from leaf_spring_analytical import LeafSpringAnalytical, E_MODULUS


class LeafSpringFEMSolver:
    """
    Finite Element System Solver for curved automotive leaf spring structures.
    Orchestrates domain discretization, stiffness assembly, boundary enforcement,
    and linear system solution.
    """

    def __init__(self, config=None, config_dir=None, analytical_engine=None):
        if config is None:
            self.cfg_mgr = ConfigManager(config_dir=config_dir)
        else:
            self.cfg_mgr = config

        if analytical_engine is None:
            self.analytical = LeafSpringAnalytical(config=self.cfg_mgr)
        else:
            self.analytical = analytical_engine

        # Geometric defaults from configuration
        dims = self.cfg_mgr.get_leaf_spring_dimensions()
        self.default_span = dims["span"]
        self.default_camber = dims["camber"]
        self.default_width = dims["width"]
        self.default_thickness = dims["thickness"]

        # Default static vertical load from config
        veh = self.cfg_mgr.get_vehicle_properties()
        self.default_f_vertical = -float(veh["load_per_spring_n"])

    def solve_system(self, K_global, F_ext, fixed_dofs):
        """
        Solves the partitioned linear equilibrium system:
        [K_free] {U_free} = {F_free}
        """
        total_dofs = len(F_ext)
        free_dofs = np.setdiff1d(np.arange(total_dofs), fixed_dofs)

        # Partition free submatrices
        K_ff = K_global[np.ix_(free_dofs, free_dofs)]
        F_f = F_ext[free_dofs]

        # Solve system of equations
        U = np.zeros(total_dofs)
        U[free_dofs] = np.linalg.solve(K_ff, F_f)

        # Reaction forces: {R} = [K] {U} - {F_ext}
        reactions = K_global @ U - F_ext

        # Total elastic strain energy
        strain_energy = 0.5 * float(U @ K_global @ U)

        return U, reactions, strain_energy

    @staticmethod
    def compute_reaction_forces(K_global, U, F_ext):
        """Computes support reaction forces at all degrees of freedom."""
        return K_global @ U - F_ext

    @staticmethod
    def compute_total_strain_energy(K_global, U):
        """Computes total integrated elastic strain energy: 0.5 * U^T * K * U."""
        return 0.5 * float(U @ K_global @ U)

    def solve(self, num_elements=40, F_vertical=None, F_horizontal=0.0,
              thickness_func=None, E_modulus=None, span=None, camber=None, width=None):
        """
        Executes end-to-end FEM analysis:
          1. Discretizes domain with curved beam elements
          2. Assembles global stiffness matrix and load vector
          3. Solves for nodal displacements, reactions, and strain energy
        """
        if F_vertical is None:
            F_vertical = self.default_f_vertical
        if E_modulus is None:
            E_modulus = self.analytical.E_modulus
        if span is None:
            span = self.default_span
        if camber is None:
            camber = self.default_camber
        if width is None:
            width = self.default_width

        # 1. Domain Discretization
        nodes, elements, element_props = build_curved_leaf_spring_mesh(
            num_elements=num_elements, span=span, camber=camber, width=width,
            thickness_func=thickness_func
        )
        num_nodes = len(nodes)
        total_dofs = 3 * num_nodes
        center_node = num_nodes // 2

        # 2. Stiffness Assembly & Load Application
        K_global = self.analytical.assemble_global_stiffness(nodes, elements, element_props, E_modulus=E_modulus)
        F_ext = self.analytical.create_global_force_vector(total_dofs, center_node, F_vertical, F_horizontal)
        fixed_dofs, free_dofs = self.analytical.identify_boundary_dofs(num_nodes)

        # 3. Solve System
        U, reactions, strain_energy = self.solve_system(K_global, F_ext, fixed_dofs)

        # Center vertical deflection (magnitude in mm)
        center_deflection_mm = abs(U[3 * center_node + 1]) * 1000.0

        return {
            "nodes": nodes,
            "elements": elements,
            "element_props": element_props,
            "num_nodes": num_nodes,
            "num_elements": len(elements),
            "total_dofs": total_dofs,
            "center_node": center_node,
            "K_global": K_global,
            "F_ext": F_ext,
            "fixed_dofs": fixed_dofs,
            "free_dofs": free_dofs,
            "U": U,
            "reactions": reactions,
            "strain_energy_J": strain_energy,
            "center_deflection_mm": center_deflection_mm,
            "F_vertical": F_vertical,
            "F_horizontal": F_horizontal,
            "E_modulus": E_modulus
        }


# -----------------------------------------------------------------------------
# Module-Level Backward-Compatible API & Singletons
# -----------------------------------------------------------------------------
_default_solver = LeafSpringFEMSolver()


def solve_system(K_global, F_ext, fixed_dofs):
    return _default_solver.solve_system(K_global, F_ext, fixed_dofs)


def compute_reaction_forces(K_global, U, F_ext):
    return LeafSpringFEMSolver.compute_reaction_forces(K_global, U, F_ext)


def compute_total_strain_energy(K_global, U):
    return LeafSpringFEMSolver.compute_total_strain_energy(K_global, U)


def solve_leaf_spring_fem(num_elements=40, F_vertical=-8240.40, F_horizontal=0.0,
                          thickness_func=None, E_modulus=E_MODULUS,
                          span=1.200, camber=0.100, width=0.060):
    return _default_solver.solve(
        num_elements=num_elements, F_vertical=F_vertical, F_horizontal=F_horizontal,
        thickness_func=thickness_func, E_modulus=E_modulus, span=span, camber=camber, width=width
    )


if __name__ == "__main__":
    print("=" * 70)
    print(" 3. LEAF SPRING FEM SOLVER CLASS & MODULE VERIFICATION")
    print("=" * 70)
    solver = LeafSpringFEMSolver()
    res = solver.solve(num_elements=40, F_vertical=-8240.40, F_horizontal=-3500.0)
    print(f"Mesh Discretization: {res['num_elements']} elements, {res['num_nodes']} nodes ({res['total_dofs']} DOFs)")
    print(f"Applied Center Load: F_v = {res['F_vertical']:.2f} N, F_h = {res['F_horizontal']:.2f} N")
    print(f"Center Deflection:   delta = {res['center_deflection_mm']:.2f} mm")
    print(f"Total Strain Energy: U_strain = {res['strain_energy_J']:.3f} J (N*m)")
    print(f"Reactions: Front Pinned Ry = {res['reactions'][1]:.2f} N, Rx = {res['reactions'][0]:.2f} N")
    print(f"           Rear Roller   Ry = {res['reactions'][3*(res['num_nodes']-1)+1]:.2f} N")
    print("\n[OK] LeafSpringFEMSolver class and module execution verified.")
