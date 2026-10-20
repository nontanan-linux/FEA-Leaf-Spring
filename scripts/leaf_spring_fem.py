"""
===============================================================================
Master Orchestrator: Automotive Leaf Spring Finite Element Method (FEM) Pipeline
Course: Finite Element Method in Engineering (M.Eng)
Instructor: Asst. Prof. Dr. Monsak Pimsarn
===============================================================================
This master script orchestrates the 5 sequential stages of the Finite Element Method:
  1. generate_mesh.py          -> Step 1: CAD Geometry & Domain Discretization
  2. leaf_spring_analytical.py -> Step 2: Element Equations & Global Stiffness Assembly
  3. fem_solve.py              -> Step 3: Boundary Enforcement & Linear Equation Solver
  4. postprocessing.py         -> Step 4: Solution Field Recovery & ANSYS Visualizations
  5. refine.py                 -> Step 5: Adaptive Refinement & Mesh Convergence Study
===============================================================================
"""

import os
import sys
import argparse

# Ensure scripts directory is in python path
scripts_dir = os.path.dirname(os.path.abspath(__file__))
project_dir = os.path.dirname(scripts_dir)
output_dir = os.path.join(project_dir, 'results')
os.makedirs(output_dir, exist_ok=True)

if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)

# -----------------------------------------------------------------------------
# Module Imports (Sequential FEM Stages 1 to 5)
# -----------------------------------------------------------------------------
# Step 1: Domain Discretization
from generate_mesh import (
    build_curved_leaf_spring_mesh,
    load_or_generate_mesh,
    calculate_mesh_diagnostics
)

# Step 2: Analytical Formulations, Material Constants & Element Assembly
from leaf_spring_analytical import (
    E_MODULUS, G_MODULUS, NU, RHO, YIELD_STRENGTH, TENSILE_STRENGTH,
    calculate_vehicle_loads,
    calculate_leaf_spring_mechanics,
    compute_element_stiffness,
    compute_transformation_matrix,
    assemble_global_stiffness,
    create_global_force_vector,
    identify_boundary_dofs
)

# Step 3: System Solver
from fem_solve import (
    solve_system,
    compute_reaction_forces,
    compute_total_strain_energy,
    solve_leaf_spring_fem
)

# Step 4: Post-Processing & Solution Fields
from postprocessing import (
    recover_element_solutions,
    print_solution_summary,
    plot_ansys_workbench_solutions,
    plot_model_revision_comparison
)

# Step 5: Adaptive Refinement & Convergence
from refine import (
    refine_longest_edge,
    plot_mesh_convergence,
    run_mesh_convergence_study
)


# -----------------------------------------------------------------------------
# Multi-Load Cases Structural Analysis Orchestration
# -----------------------------------------------------------------------------
def run_load_cases_analysis(num_elements=60):
    """
    Executes structural analysis for 4 standard vehicle suspension load conditions:
      - Case 1: Normal Static Vertical Load (1G Laden Ride)
      - Case 2: Peak Dynamic Bump Load (2.0G Road Impact)
      - Case 3: Combined Bump + 0.8G Braking Longitudinal Load
      - Case 4: Severe Dynamic Bump + Maximum Longitudinal Braking
    
    Generates the executive ANSYS Workbench 7-solution dashboard for Case 1.
    """
    print("\n" + "=" * 70)
    print(" 4. MULTI-LOAD CASE STRUCTURAL ANALYSIS (SOLID SPRING STEEL)")
    print("=" * 70)

    cases = [
        ("Case 1: Normal Static Vertical Load", -8240.40, 0.0),
        ("Case 2: Peak Dynamic Bump Load (2.0G)", -16480.80, 0.0),
        ("Case 3: Combined Bump + Braking Load", -8240.40, -3500.0),
        ("Case 4: Severe Dynamic Bump + Braking", -16480.80, -5000.0),
    ]

    study_results = []

    for idx_c, (case_name, F_v, F_h) in enumerate(cases):
        # Step 3: Solve linear boundary value system
        raw_res = solve_leaf_spring_fem(
            num_elements=num_elements,
            F_vertical=F_v,
            F_horizontal=F_h
        )

        # Step 4: Post-process 7 FEA solution fields
        res = recover_element_solutions(raw_res)
        print_solution_summary(res, case_name)

        study_results.append({
            "case": case_name,
            "F_v": F_v,
            "F_h": F_h,
            "max_stress": res["max_von_mises_MPa"],
            "deflection": res["center_deflection_mm"],
            "FoS": res["FoS"],
            "status": "SAFE" if res["FoS"] >= 1.0 else "EXCEEDS YIELD",
            "res": res
        })

        if idx_c == 0:
            # Step 4: Generate ANSYS Workbench 8-panel dashboard for baseline ride
            ansys_path = os.path.join(output_dir, "ansys_workbench_solutions.png")
            plot_ansys_workbench_solutions(res, output_path=ansys_path)

    return study_results


# -----------------------------------------------------------------------------
# Model Revision: Uniform vs. Parabolic Tapered Leaf Spring
# -----------------------------------------------------------------------------
def run_model_revision(num_elements=60):
    """
    Performs design revision study comparing:
      - Baseline: Uniform 12 mm thickness leaf spring
      - Revised: Parabolic tapered leaf spring (14.5 mm center down to 7.5 mm eye ends)
    
    Evaluates mass reduction, peak stress reduction, and factor of safety improvements.
    Generates comparison stress distribution plot.
    """
    print("\n" + "=" * 70)
    print(" 5. MODEL REVISION: UNIFORM THICKNESS vs. PARABOLIC TAPERED LEAF SPRING")
    print("=" * 70)

    span = 1.200

    # Parabolic tapering function: t(x) = t_center - (t_center - t_eye) * (2x/span)^2
    def parabolic_taper(x):
        ratio = abs(2.0 * x / span)
        return 0.0145 - (0.0145 - 0.0075) * (ratio**2)

    F_bump = -16480.80  # 2.0G dynamic road bump

    # Solve baseline uniform leaf spring
    raw_baseline = solve_leaf_spring_fem(num_elements=num_elements, F_vertical=F_bump, thickness_func=None)
    res_baseline = recover_element_solutions(raw_baseline)

    # Solve revised parabolic tapered leaf spring
    raw_revised = solve_leaf_spring_fem(num_elements=num_elements, F_vertical=F_bump, thickness_func=parabolic_taper)
    res_revised = recover_element_solutions(raw_revised)

    print("BASELINE MODEL (Uniform 12 mm):")
    print(f"  Mass: {res_baseline['total_mass_kg']:.2f} kg | Max Stress: {res_baseline['max_von_mises_MPa']:.2f} MPa | "
          f"Deflection: {res_baseline['center_deflection_mm']:.2f} mm | FoS: {res_baseline['FoS']:.2f}")

    print("\nREVISED MODEL (Parabolic Tapered 14.5 -> 7.5 mm):")
    print(f"  Mass: {res_revised['total_mass_kg']:.2f} kg | Max Stress: {res_revised['max_von_mises_MPa']:.2f} MPa | "
          f"Deflection: {res_revised['center_deflection_mm']:.2f} mm | FoS: {res_revised['FoS']:.2f}")

    mass_saving = (res_baseline['total_mass_kg'] - res_revised['total_mass_kg']) / res_baseline['total_mass_kg'] * 100.0
    stress_reduction = (res_baseline['max_von_mises_MPa'] - res_revised['max_von_mises_MPa']) / res_baseline['max_von_mises_MPa'] * 100.0

    print("\n>>> REVISION SUMMARY:")
    print(f"  Mass Reduction:    {mass_saving:.1f}%")
    print(f"  Stress Reduction:  {stress_reduction:.1f}%")
    print(f"  FoS Improvement:   {res_baseline['FoS']:.2f} -> {res_revised['FoS']:.2f}")

    # Plot Comparison Chart
    comp_path = os.path.join(output_dir, "model_revision_comparison.png")
    plot_model_revision_comparison(res_baseline, res_revised, output_path=comp_path)

    return res_baseline, res_revised


# -----------------------------------------------------------------------------
# Main Pipeline CLI Entry Point
# -----------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Finite Element Method (FEM) Automotive Leaf Spring Master Analysis Pipeline"
    )
    parser.add_argument("--all", action="store_true", help="Run full pipeline: mesh study, load cases, and revision")
    parser.add_argument("--refine", action="store_true", help="Run Step 5 mesh convergence study only")
    parser.add_argument("--solve", action="store_true", help="Run Steps 3 & 4 multi-load cases analysis only")
    parser.add_argument("--revision", action="store_true", help="Run Model Revision comparison only")
    args = parser.parse_args()

    # Default to running all stages if no specific flag is provided
    run_all = args.all or not (args.refine or args.solve or args.revision)

    print("=" * 80)
    print(" AUTOMOTIVE LEAF SPRING - FINITE ELEMENT METHOD (FEM) ANALYSIS PIPELINE")
    print("=" * 80)

    if run_all or args.refine:
        # Step 5: Adaptive Refinement & Mesh Convergence Study
        run_mesh_convergence_study()

    if run_all or args.solve:
        # Steps 3 & 4: Multi-Load Cases Structural Solution & ANSYS Dashboard
        run_load_cases_analysis()

    if run_all or args.revision:
        # Model Revision: Uniform vs Parabolic Tapered Optimization
        run_model_revision()

    print("\n" + "=" * 80)
    print(" FEM PIPELINE EXECUTION COMPLETED SUCCESSFULLY")
    print(f" Generated artifacts located in: {output_dir}")
    print("=" * 80)


if __name__ == "__main__":
    main()
