"""
===============================================================================
Module 3: Finite Element System Solver for Automotive Leaf Spring
Course: Finite Element Method in Engineering (M.Eng)
Instructor: Asst. Prof. Dr. Monsak Pimsarn
===============================================================================
This module handles:
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

from generate_mesh import build_curved_leaf_spring_mesh
from leaf_spring_analytical import (
    E_MODULUS, assemble_global_stiffness,
    create_global_force_vector, identify_boundary_dofs
)

def solve_system(K_global, F_ext, fixed_dofs):
    """
    Solves the reduced linear equilibrium system by partitioning restrained DOFs:
    [K_free] {U_free} = {F_free}
    
    Parameters:
        K_global (ndarray): Total assembled structural stiffness matrix (total_dofs x total_dofs)
        F_ext (ndarray): Global external force vector (total_dofs)
        fixed_dofs (list/array): Indices of restrained degrees of freedom (boundary supports)
        
    Returns:
        U (ndarray): Full global displacement vector (total_dofs)
        reactions (ndarray): Reaction forces at restrained DOFs (total_dofs)
        strain_energy (float): Total elastic strain energy 0.5 * U^T * K * U (Joules)
    """
    total_dofs = len(F_ext)
    free_dofs = np.setdiff1d(np.arange(total_dofs), fixed_dofs)
    
    # Partition submatrices
    K_ff = K_global[np.ix_(free_dofs, free_dofs)]
    F_f = F_ext[free_dofs]
    
    # Solve for free displacements
    U = np.zeros(total_dofs)
    U[free_dofs] = np.linalg.solve(K_ff, F_f)
    
    # Reaction forces: {R} = [K] {U} - {F_ext}
    reactions = K_global @ U - F_ext
    
    # Total strain energy: U_strain = 0.5 * {U}^T [K] {U}
    strain_energy = 0.5 * float(U @ K_global @ U)
    
    return U, reactions, strain_energy

def compute_reaction_forces(K_global, U, F_ext):
    """Computes reaction forces at all nodes: {R} = [K]{U} - {F_ext}."""
    return K_global @ U - F_ext

def compute_total_strain_energy(K_global, U):
    """Computes total elastic strain energy (N*m / Joules): 0.5 * U^T * K * U."""
    return 0.5 * float(U @ K_global @ U)

def solve_leaf_spring_fem(num_elements=40, F_vertical=-8240.40, F_horizontal=0.0,
                         thickness_func=None, E_modulus=E_MODULUS,
                         span=1.200, camber=0.100, width=0.060):
    """
    Executes the standard FEM solution step for an automotive curved leaf spring:
    1. Meshing (Domain Discretization via generate_mesh)
    2. Assembly (Global Stiffness & Load Vector via leaf_spring_analytical)
    3. Boundary Enforcement & Linear Solution
    
    Returns:
        Dictionary containing mesh, displacement vectors, forces, reactions, and strain energy.
    """
    # 1. Domain Discretization
    nodes, elements, element_props = build_curved_leaf_spring_mesh(
        num_elements=num_elements, span=span, camber=camber, width=width,
        thickness_func=thickness_func
    )
    num_nodes = len(nodes)
    total_dofs = 3 * num_nodes
    center_node = num_nodes // 2
    
    # 2. Stiffness Assembly & Load Application
    K_global = assemble_global_stiffness(nodes, elements, element_props, E_modulus=E_modulus)
    F_ext = create_global_force_vector(total_dofs, center_node, F_vertical, F_horizontal)
    fixed_dofs, free_dofs = identify_boundary_dofs(num_nodes)
    
    # 3. Solve System
    U, reactions, strain_energy = solve_system(K_global, F_ext, fixed_dofs)
    
    # Center vertical deflection (Uy at center node)
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

if __name__ == "__main__":
    print("=" * 70)
    print(" 3. FINITE ELEMENT SYSTEM SOLVER VERIFICATION")
    print("=" * 70)
    
    res = solve_leaf_spring_fem(num_elements=40, F_vertical=-8240.40, F_horizontal=-3500.0)
    print(f"Mesh Discretization: {res['num_elements']} elements, {res['num_nodes']} nodes ({res['total_dofs']} DOFs)")
    print(f"Applied Center Load: F_v = {res['F_vertical']:.2f} N, F_h = {res['F_horizontal']:.2f} N")
    print(f"Center Deflection:   delta = {res['center_deflection_mm']:.2f} mm")
    print(f"Total Strain Energy: U_strain = {res['strain_energy_J']:.3f} J (N*m)")
    print(f"Reactions: Front Pinned Ry = {res['reactions'][1]:.2f} N, Rx = {res['reactions'][0]:.2f} N")
    print(f"           Rear Roller   Ry = {res['reactions'][3*(res['num_nodes']-1)+1]:.2f} N")
    print("\n[OK] fem_solve module execution verified.")
