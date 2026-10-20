"""
===============================================================================
Master Orchestrator: Automotive Leaf Spring Finite Element Method (FEM) Pipeline
Course: Finite Element Method in Engineering (M.Eng)
Instructor: Asst. Prof. Dr. Monsak Pimsarn
===============================================================================
This master script provides the Object-Oriented LeafSpringFEMPipeline orchestrating:
  1. config_loader.py          -> Central Configuration Management (config/*.yaml)
  2. generate_mesh.py          -> Step 1: CAD Geometry & Domain Discretization
  3. leaf_spring_analytical.py -> Step 2: Element Equations & Global Stiffness Assembly
  4. fem_solve.py              -> Step 3: Boundary Enforcement & Linear Equation Solver
  5. postprocessing.py         -> Step 4: Solution Field Recovery & ANSYS Visualizations
  6. refine.py                 -> Step 5: Adaptive Refinement & Mesh Convergence Study
===============================================================================
"""

import os
import sys
import argparse

# Ensure scripts directory is in python path
scripts_dir = os.path.dirname(os.path.abspath(__file__))
if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)

# Central Configuration Manager
from config_loader import ConfigManager, config_manager

# Step 1: Domain Discretization
from generate_mesh import (
    build_curved_leaf_spring_mesh,
    load_or_generate_mesh,
    calculate_mesh_diagnostics
)

# Step 2: Analytical Formulations, Material Constants & Element Assembly
from leaf_spring_analytical import (
    LeafSpringAnalytical,
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
    LeafSpringFEMSolver,
    solve_system,
    compute_reaction_forces,
    compute_total_strain_energy,
    solve_leaf_spring_fem
)

# Step 4: Post-Processing & Solution Fields
from postprocessing import (
    LeafSpringPostProcessor,
    recover_element_solutions,
    print_solution_summary,
    plot_ansys_workbench_solutions,
    plot_model_revision_comparison
)

# Step 5: Adaptive Refinement & Convergence
from refine import (
    MeshConvergenceStudy,
    refine_longest_edge,
    plot_mesh_convergence,
    run_mesh_convergence_study
)


class LeafSpringFEMPipeline:
    """
    Master Finite Element Method Pipeline Orchestrator.
    Connects configuration inputs, mesh generation, analytical assembly,
    system solution, post-processing field recovery, and mesh convergence study.
    """

    def __init__(self, config_dir=None):
        self.cfg_mgr = ConfigManager(config_dir=config_dir)
        self.output_dir = os.path.join(self.cfg_mgr.project_dir, 'results')
        os.makedirs(self.output_dir, exist_ok=True)

        # Initialize modular engines
        self.analytical = LeafSpringAnalytical(config=self.cfg_mgr)
        self.solver = LeafSpringFEMSolver(config=self.cfg_mgr, analytical_engine=self.analytical)
        self.postprocessor = LeafSpringPostProcessor(config=self.cfg_mgr)
        self.convergence = MeshConvergenceStudy(config=self.cfg_mgr)

        # Load operational cases and dimensions from config
        self.load_cases = self.cfg_mgr.get_load_cases()
        self.dimensions = self.cfg_mgr.get_leaf_spring_dimensions()

    # -------------------------------------------------------------------------
    # 1. Step 5: Adaptive Refinement & Mesh Convergence Study
    # -------------------------------------------------------------------------
    def run_convergence_study(self, output_path=None):
        """Executes Step 5: Mesh convergence study loading stages from config."""
        if output_path is None:
            output_path = os.path.join(self.output_dir, "mesh_convergence_study.png")
        return self.convergence.run_study(output_path=output_path)

    # -------------------------------------------------------------------------
    # 2. Steps 3 & 4: Multi-Load Cases Structural Analysis
    # -------------------------------------------------------------------------
    def run_load_cases_analysis(self, num_elements=60):
        """
        Executes structural analysis for all operating conditions defined in config:
          - Dynamic load cases from config/vehicle_information.yaml
          - Solves boundary equilibrium via Step 3 (LeafSpringFEMSolver)
          - Recovers all 7 FEA solution fields via Step 4 (LeafSpringPostProcessor)
          - Generates ANSYS Workbench 8-panel dashboard for baseline ride
        """
        print("\n" + "=" * 70)
        print(" 4. MULTI-LOAD CASE STRUCTURAL ANALYSIS (FROM CONFIG)")
        print("=" * 70)

        study_results = []

        for idx_c, case in enumerate(self.load_cases):
            case_name = case["name"]
            F_v = case["F_vertical"]
            F_h = case["F_horizontal"]

            # Step 3: Solve linear boundary value system
            raw_res = self.solver.solve(
                num_elements=num_elements,
                F_vertical=F_v,
                F_horizontal=F_h
            )

            # Step 4: Post-process 7 FEA solution fields
            res = self.postprocessor.recover_element_solutions(raw_res)
            self.postprocessor.print_solution_summary(res, case_name)

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
                ansys_path = os.path.join(self.output_dir, "ansys_workbench_solutions.png")
                self.postprocessor.plot_ansys_workbench_solutions(res, output_path=ansys_path)

        return study_results

    # -------------------------------------------------------------------------
    # 3. Model Revision Optimization (Uniform vs. Parabolic Tapered)
    # -------------------------------------------------------------------------
    def run_model_revision(self, num_elements=60):
        """
        Performs design revision study comparing:
          - Baseline: Uniform 12 mm thickness leaf spring
          - Revised: Parabolic tapered leaf spring (14.5 mm center down to 7.5 mm eye ends)
        """
        print("\n" + "=" * 70)
        print(" 5. MODEL REVISION: UNIFORM THICKNESS vs. PARABOLIC TAPERED LEAF SPRING")
        print("=" * 70)

        span = self.dimensions["span"]

        # Parabolic tapering function: t(x) = t_center - (t_center - t_eye) * (2x/span)^2
        def parabolic_taper(x):
            ratio = abs(2.0 * x / span)
            return 0.0145 - (0.0145 - 0.0075) * (ratio**2)

        # Dynamic 2.0G bump impact load
        veh = self.cfg_mgr.get_vehicle_properties()
        F_bump = -float(veh["load_per_spring_n"]) * 2.0

        # Solve baseline uniform leaf spring
        raw_baseline = self.solver.solve(num_elements=num_elements, F_vertical=F_bump, thickness_func=None)
        res_baseline = self.postprocessor.recover_element_solutions(raw_baseline)

        # Solve revised parabolic tapered leaf spring
        raw_revised = self.solver.solve(num_elements=num_elements, F_vertical=F_bump, thickness_func=parabolic_taper)
        res_revised = self.postprocessor.recover_element_solutions(raw_revised)

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
        comp_path = os.path.join(self.output_dir, "model_revision_comparison.png")
        self.postprocessor.plot_model_revision_comparison(res_baseline, res_revised, output_path=comp_path)

        return res_baseline, res_revised

    # -------------------------------------------------------------------------
    # 4. Master Execution Flow
    # -------------------------------------------------------------------------
    def run_all(self, num_elements=60):
        """Executes full sequential FEM analysis pipeline."""
        self.run_convergence_study()
        self.run_load_cases_analysis(num_elements=num_elements)
        self.run_model_revision(num_elements=num_elements)


# -----------------------------------------------------------------------------
# Module-Level Backward-Compatible API & Singletons
# -----------------------------------------------------------------------------
_default_pipeline = LeafSpringFEMPipeline()


def run_load_cases_analysis(num_elements=60):
    return _default_pipeline.run_load_cases_analysis(num_elements=num_elements)


def run_model_revision(num_elements=60):
    return _default_pipeline.run_model_revision(num_elements=num_elements)


# -----------------------------------------------------------------------------
# Main CLI Entry Point
# -----------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Finite Element Method (FEM) Automotive Leaf Spring Master Analysis Pipeline"
    )
    parser.add_argument("--all", action="store_true", help="Run full pipeline: mesh study, load cases, and revision")
    parser.add_argument("--refine", action="store_true", help="Run Step 5 mesh convergence study only")
    parser.add_argument("--solve", action="store_true", help="Run Steps 3 & 4 multi-load cases analysis only")
    parser.add_argument("--revision", action="store_true", help="Run Model Revision comparison only")
    parser.add_argument("--elements", type=int, default=60, help="Number of finite elements (default: 60)")
    args = parser.parse_args()

    pipeline = LeafSpringFEMPipeline()

    run_all = args.all or not (args.refine or args.solve or args.revision)

    print("=" * 80)
    print(" AUTOMOTIVE LEAF SPRING - FINITE ELEMENT METHOD (FEM) ANALYSIS PIPELINE")
    print("=" * 80)

    if run_all or args.refine:
        pipeline.run_convergence_study()

    if run_all or args.solve:
        pipeline.run_load_cases_analysis(num_elements=args.elements)

    if run_all or args.revision:
        pipeline.run_model_revision(num_elements=args.elements)

    print("\n" + "=" * 80)
    print(" FEM PIPELINE EXECUTION COMPLETED SUCCESSFULLY")
    print(f" Generated artifacts located in: {pipeline.output_dir}")
    print("=" * 80)


if __name__ == "__main__":
    main()
