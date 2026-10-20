"""
===============================================================================
Module 4: Post-Processing & Solution Recovery for Automotive Leaf Spring FEM
Course: Finite Element Method in Engineering (M.Eng)
Instructor: Asst. Prof. Dr. Monsak Pimsarn
===============================================================================
This module provides the Object-Oriented class LeafSpringPostProcessor:
1. Element strain and stress recovery from nodal displacements:
   - Axial normal forces & bending moment distribution
   - 7 Standard FEA Solution Field Quantities:
     1. Total Deformation (U_total, mm)
     2. Directional Deformation in Y-Axis (U_y, mm)
     3. Equivalent Elastic Strain (eps_eq, um/m microstrain)
     4. Equivalent Von Mises Stress (sigma_vm, MPa)
     5. Maximum Principal Stress (sigma_1, MPa)
     6. Maximum Shear Stress / Tresca (tau_max, MPa)
     7. Elastic Strain Energy Distribution (U_strain, N*m / J)
2. Safety factor evaluation (FoS = sigma_yield / sigma_vm_max) & total spring mass
3. Executive 8-Panel ANSYS Workbench Solution Dashboard generation
4. Model revision comparison plotting (Uniform vs. Parabolic Tapered Leaf Spring)
===============================================================================
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

# Ensure scripts directory is in python path
scripts_dir = os.path.dirname(os.path.abspath(__file__))
if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)

from config_loader import ConfigManager
from leaf_spring_analytical import LeafSpringAnalytical, E_MODULUS, G_MODULUS, RHO, YIELD_STRENGTH


class LeafSpringPostProcessor:
    """
    Finite Element Post-Processor recovering internal forces, stresses, strains,
    energy distributions, and generating publication visualizations.
    """

    def __init__(self, config=None, config_dir=None):
        if config is None:
            self.cfg_mgr = ConfigManager(config_dir=config_dir)
        else:
            self.cfg_mgr = config

        mat = self.cfg_mgr.get_material_properties()
        self.E_modulus = mat["E_modulus"]
        self.G_modulus = mat["G_modulus"]
        self.rho = mat["rho"]
        self.yield_strength = mat["yield_strength"]

        # Output directory configuration
        solver_cfg = getattr(self.cfg_mgr, "solver_cfg", {})
        post_cfg = solver_cfg.get("post_processing", {})
        raw_out = post_cfg.get("results_output_dir", "results")
        clean_out = raw_out.replace("../", "").replace("..\\", "")
        self.output_dir = os.path.normpath(os.path.join(self.cfg_mgr.project_dir, clean_out))
        os.makedirs(self.output_dir, exist_ok=True)

    def recover_element_solutions(self, solve_res, E_modulus=None, G_modulus=None,
                                  rho=None, yield_strength=None):
        """
        Recovers element-level stress, strain, moment, and energy distributions
        from global displacement vector U.
        """
        if E_modulus is None:
            E_modulus = self.E_modulus
        if G_modulus is None:
            G_modulus = self.G_modulus
        if rho is None:
            rho = self.rho
        if yield_strength is None:
            yield_strength = self.yield_strength

        nodes = solve_res["nodes"]
        elements = solve_res["elements"]
        element_props = solve_res["element_props"]
        U = solve_res["U"]
        center_node = solve_res["center_node"]

        # 1. Nodal Displacements (mm)
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

            T = LeafSpringAnalytical.compute_transformation_matrix(c, s)
            dofs = [3*n1, 3*n1+1, 3*n1+2, 3*n2, 3*n2+1, 3*n2+2]
            u_elem_local = T @ U[dofs]

            # Internal axial force and bending moments
            N_axial = (E_modulus * A_el / L_el) * (u_elem_local[3] - u_elem_local[0])
            M1 = (E_modulus * I_el / L_el**2) * (6.0*(u_elem_local[1] - u_elem_local[4]) + L_el*(4.0*u_elem_local[2] + 2.0*u_elem_local[5]))
            M2 = (E_modulus * I_el / L_el**2) * (6.0*(u_elem_local[1] - u_elem_local[4]) + L_el*(2.0*u_elem_local[2] + 4.0*u_elem_local[5]))
            M_max = max(abs(M1), abs(M2))

            # 4. Equivalent Von Mises Stress (MPa)
            sigma_axial = N_axial / A_el
            sigma_bending = M_max / Z_el
            sigma_von_mises = abs(sigma_axial) + sigma_bending

            # 5. Maximum Principal Stress (Peak tensile normal stress)
            sigma_principal_1 = max(0.0, sigma_axial + sigma_bending, -sigma_axial + sigma_bending)

            # 6. Maximum Shear Stress (Tresca: tau_max = sigma_1 / 2)
            tau_max = sigma_principal_1 / 2.0

            # 3. Equivalent Elastic Strain (eps_eq = sigma_vm / E)
            strain_eq = sigma_von_mises / E_modulus
            shear_strain = tau_max / G_modulus

            # 7. Element Strain Energy (N*m / Joules)
            e_strain_energy = (L_el / (6.0 * E_modulus * I_el)) * (M1**2 + M1 * M2 + M2**2) + (N_axial**2 * L_el) / (2.0 * E_modulus * A_el)

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

        total_volume = sum(p["A"] * p["length"] for p in element_props)
        total_mass = total_volume * rho
        max_vm_stress = np.max(element_stresses)

        res = dict(solve_res)
        res.update({
            "u_x_nodes_mm": u_x_nodes_mm,
            "u_y_nodes_mm": u_y_nodes_mm,
            "u_total_nodes_mm": u_total_nodes_mm,
            "element_u_total_mm": element_u_total,
            "element_u_y_mm": element_u_y,
            "max_von_mises_MPa": max_vm_stress / 1e6,
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
            "total_mass_kg": total_mass,
            "FoS": yield_strength / max_vm_stress
        })
        return res

    def print_solution_summary(self, res, title="4. MULTI-LOAD CASE STRUCTURAL ANALYSIS"):
        """Prints formatted tabular summary of the 7 FEA solution metrics."""
        status = "SAFE" if res["FoS"] >= 1.0 else "EXCEEDS YIELD"
        print(f"\n[{title}]")
        print(f"  1. Total Deformation:              {res['center_deflection_mm']:8.2f} mm")
        print(f"  2. Directional Def. (Y-Axis):     -{res['center_deflection_mm']:8.2f} mm")
        print(f"  3. Equivalent Elastic Strain:     {res['max_microstrain']:8.1f} um/m (microstrain)")
        print(f"  4. Equivalent Stress (Von Mises): {res['max_von_mises_MPa']:6.2f} MPa (Yield: {self.yield_strength/1e6:.0f} MPa)")
        print(f"  5. Maximum Principal Stress:      {res['max_principal_stress_MPa']:6.2f} MPa")
        print(f"  6. Maximum Shear Stress (Tresca): {res['max_shear_stress_MPa']:6.2f} MPa")
        print(f"  7. Total Strain Energy:            {res['strain_energy_J']:8.2f} N.m (J)")
        print(f"  >> Factor of Safety (FoS):          {res['FoS']:8.2f} -> {status}")

    def plot_ansys_workbench_solutions(self, res, output_path=None):
        """
        Generates executive 8-panel dashboard corresponding to the 7 standard
        ANSYS Workbench Mechanical solution fields + Solution Summary Dashboard.
        """
        if output_path is None:
            output_path = os.path.join(self.output_dir, "ansys_workbench_solutions.png")

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
                "unit": "um/m",
                "subtitle": f"Von Mises Strain | eps_eq = sigma_vm / E (Yield: {self.yield_strength/self.E_modulus*1e6:.0f} um/m)",
                "cmap": "turbo",
                "fmt": "{:.1f}"
            },
            {
                "data": res["element_stresses_MPa"],
                "title": "4. Equivalent Stress (Von Mises)",
                "unit": "MPa",
                "subtitle": f"Octahedral Distortion Metric (Yield: {self.yield_strength/1e6:.0f} MPa)",
                "cmap": "turbo",
                "fmt": "{:.1f}"
            },
            {
                "data": res["element_principal_1_MPa"],
                "title": "5. Maximum Principal Stress (sigma_1)",
                "unit": "MPa",
                "subtitle": "Peak Tensile Normal Stress (Fatigue Crack Initiation)",
                "cmap": "turbo",
                "fmt": "{:.1f}"
            },
            {
                "data": res["element_tau_max_MPa"],
                "title": "6. Maximum Shear Stress (Tresca)",
                "unit": "MPa",
                "subtitle": f"Tresca Metric: tau_max = sigma_1/2 | Peak gamma = {res['max_shear_microstrain']:.1f} um/m",
                "cmap": "turbo",
                "fmt": "{:.1f}"
            },
            {
                "data": res["element_strain_energy_J"],
                "title": "7. Strain Energy Distribution",
                "unit": "N*m (J)",
                "subtitle": f"Elastic Energy per Element | Total: {res['strain_energy_J']:.2f} N*m",
                "cmap": "turbo",
                "fmt": "{:.3f}"
            }
        ]

        for idx in range(7):
            ax = axes.flat[idx]
            sc = solution_configs[idx]
            ax.set_facecolor('#FFFFFF')
            data = sc["data"]
            val_min = float(np.min(data))
            val_max = float(np.max(data))

            ax.plot(x_orig, y_orig, color='#94A3B8', linestyle='--', linewidth=1.2, alpha=0.6, label='Undeformed')

            lc = LineCollection(segs, cmap=sc["cmap"], linewidths=7.0)
            lc.set_array(data)
            lc.set_clim(val_min, val_max)
            ax.add_collection(lc)

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
            ["6. Max Shear Stress", f"{res['max_shear_stress_MPa']:.2f} MPa", "45 deg Shear Plane"],
            ["7. Total Strain Energy", f"{res['strain_energy_J']:.2f} N*m", "Full Spring Span"]
        ]

        col_labels = ["ANSYS Solution Item", "Max Value", "Critical Zone"]
        tab = ax8.table(cellText=table_data, colLabels=col_labels, loc='center', cellLoc='left',
                        colWidths=[0.42, 0.28, 0.30])
        tab.auto_set_font_size(False)
        tab.set_fontsize(8.5)
        tab.scale(1.0, 1.65)

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

        ax8.set_title("8. FEA Solution Summary Dashboard\nFull Metric Breakdown (Static Ride: Fv = -8,240.40 N)",
                      fontsize=10, fontweight='bold', pad=10)

        plt.suptitle("Finite Element Method - 7 Structural Solution Field Distribution\n"
                     "Automotive Semi-Elliptic Leaf Spring (Static Ride: F_v = -8,240.40 N | Material: 55Cr3 / SUP9 Spring Steel)",
                     fontsize=13, fontweight='bold', y=0.99)
        plt.tight_layout(rect=[0, 0.02, 1, 0.96])
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()
        print(f"\n[SAVED] 7-Solution beam field figure saved to: {output_path}")

    def plot_model_revision_comparison(self, res_baseline, res_revised, output_path=None):
        """Generates comparison chart for Uniform vs Parabolic Tapered model revision."""
        if output_path is None:
            output_path = os.path.join(self.output_dir, "model_revision_comparison.png")

        x_coords = [p["x_mid"] * 1000.0 for p in res_baseline["element_props"]]
        fig, ax = plt.subplots(figsize=(10, 4.5))
        ax.plot(x_coords, res_baseline["element_stresses_MPa"], 'r--', linewidth=2, label="Baseline (Uniform 12 mm)")
        ax.plot(x_coords, res_revised["element_stresses_MPa"], 'b-', linewidth=2, label="Revised (Parabolic Tapered)")
        ax.axhline(y=self.yield_strength/1e6, color='k', linestyle=':', label=f"Yield Strength ({self.yield_strength/1e6:.0f} MPa)")
        ax.set_title("Stress Distribution Along Leaf Spring (2.0G Bump Load)", fontsize=12, fontweight='bold')
        ax.set_xlabel("Longitudinal Position from Center (mm)", fontsize=11)
        ax.set_ylabel("Von Mises Stress (MPa)", fontsize=11)
        ax.legend()
        ax.grid(True, linestyle='--', alpha=0.6)
        plt.tight_layout()
        plt.savefig(output_path, dpi=300)
        plt.close()
        print(f"[SAVED] Comparison chart saved to: {output_path}")


# -----------------------------------------------------------------------------
# Module-Level Backward-Compatible API & Singletons
# -----------------------------------------------------------------------------
_default_postprocessor = LeafSpringPostProcessor()


def recover_element_solutions(solve_res, E_modulus=E_MODULUS, G_modulus=G_MODULUS,
                              rho=RHO, yield_strength=YIELD_STRENGTH):
    return _default_postprocessor.recover_element_solutions(
        solve_res, E_modulus=E_modulus, G_modulus=G_modulus, rho=rho, yield_strength=yield_strength
    )


def print_solution_summary(res, title="4. MULTI-LOAD CASE STRUCTURAL ANALYSIS"):
    return _default_postprocessor.print_solution_summary(res, title=title)


def plot_ansys_workbench_solutions(res, output_path=None):
    return _default_postprocessor.plot_ansys_workbench_solutions(res, output_path=output_path)


def plot_model_revision_comparison(res_baseline, res_revised, output_path=None):
    return _default_postprocessor.plot_model_revision_comparison(res_baseline, res_revised, output_path=output_path)


if __name__ == "__main__":
    from fem_solve import solve_leaf_spring_fem
    print("=" * 70)
    print(" 4. POST-PROCESSING CLASS & MODULE VERIFICATION")
    print("=" * 70)
    post = LeafSpringPostProcessor()
    raw_res = solve_leaf_spring_fem(num_elements=60, F_vertical=-8240.40)
    res = post.recover_element_solutions(raw_res)
    post.print_solution_summary(res, "Static Laden 1G Solution Recovery")
    post.plot_ansys_workbench_solutions(res)
    print("\n[OK] LeafSpringPostProcessor class and module execution verified.")
