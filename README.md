# Automotive Leaf Spring FEA Project (Python & PrePoMax)

**Course:** Finite Element Method in Engineering (M.Eng Program)  
**Instructor:** Asst. Prof. Dr. Monsak Pimsarn  
**Project Location:** `D:\M.eng\FiniteElementMethode\FEA_Leaf_Spring`  

---

## 📁 Project Structure

```
D:\M.eng\FiniteElementMethode\FEA_Leaf_Spring\
├── README.md                   # This project guide and execution instructions
├── config\                     # Project configuration YAML files
│   ├── vehicle_information.yaml# Vehicle specifications, masses, and dynamics
│   ├── solver.yaml             # Solver methods, units, material, BCs, and load cases
│   └── model.yaml              # CAD geometry, mesh parameters, and model revisions
├── resource\                   # 3D CAD & Geometry files
│   ├── Leaf Spring.stp         # 3D STEP solid model for CAD/PrePoMax import
│   ├── Leaf Spring.igs         # IGES format
│   ├── Leaf Spring.stl         # 3D STL mesh model
│   ├── Leaf Spring.jpg         # Model visual preview
│   └── leaf_spring_cad_model.jpg # Rendered CAD solid geometry image
├── results\                    # All FEA results and the academic project report
│   ├── result.md               # Full 5-section academic project report (Matches Dr. Monsak's format)
│   ├── vehicle_dynamics_fbd.png# Figure 1: Vehicle Free Body Diagram & Load Distribution
│   ├── fixed_support_model.png # Figure 3: Pure CAD 3D Fixed Support figure
│   ├── fixed_support_cad_view.png # Standalone pure CAD Fixed Support view
│   ├── fixed_support_cad_closeup.png # 3x close-up detail of scoped inner cylindrical bore
│   ├── mesh_boundary_conditions.png # Figure 4: Restraints & loading schematic
│   ├── mesh_model_3d.png       # Figure 5: 3D Finite Element Solid Mesh publication figure
│   ├── mesh_model_cad_view.png # Standalone 3D CAD mesh view in exact sample perspective
│   ├── mesh_model_cad_closeup.png # 3x zoom close-up detail of front eye mesh density
│   ├── mesh_convergence_study.png # Figure 6: Generated convergence curve chart
│   └── model_revision_comparison.png # Figure 7: Baseline vs. revised comparison chart
└── scripts\                    # Python source code for analysis & FEA
    ├── leaf_spring_analytical.py   # Vehicle dynamics load & analytical beam mechanics calculation
    ├── leaf_spring_fem.py          # Complete Python Finite Element solver & convergence plotter
    ├── plot_vehicle_dynamics.py    # Vehicle dynamics Free Body Diagram generator (Figure 1)
    ├── plot_cad_fixed_support.py   # Direct CAD 3D renderer (Open3D) & Fixed Support hole highlighter
    ├── plot_cad_mesh.py            # Direct CAD 3D FEA mesh renderer with Rivara bisection refinement
    ├── plot_mesh_model.py          # 3D STL mesh and boundary condition generator
    └── plot_fixed_support.py       # Dedicated 3D Fixed Support visualizer
```

---

## 🚀 How to Run the Python Scripts

### 1. Analytical Vehicle Dynamics & Mechanics Calculation
Computes static normal force, 2.0G dynamic bump impact, longitudinal braking weight transfer, and cornering lateral loads:
```powershell
python scripts\leaf_spring_analytical.py
```

### 2. Python Finite Element Method (FEM) Solver & Plotter
Solves the curved leaf spring frame/beam FEM, evaluates the 4 operational load cases, performs strain energy convergence, and generates the comparison charts:
```powershell
python scripts\leaf_spring_fem.py
```
*Outputs generated into `results/`:*
- `results/mesh_convergence_study.png` (Strain energy convergence graph - Figure 6)
- `results/model_revision_comparison.png` (Stress distribution comparison between Baseline and Parabolic Tapered design - Figure 7)

### 3. Generate Vehicle Dynamics Diagram & CAD Visualizations
```powershell
python scripts\plot_vehicle_dynamics.py
python scripts\plot_cad_fixed_support.py
python scripts\plot_cad_mesh.py
```
*Outputs generated into `results/`:*
- `results/vehicle_dynamics_fbd.png` (Figure 1)
- `results/fixed_support_model.png` (Figure 3)
- `results/mesh_model_3d.png` (Figure 5)

---

## 🖥️ How to Run in PrePoMax (Optional 3D Visualization)

1. Launch **PrePoMax**.
2. Click **`File`** $\rightarrow$ **`Import...`**.
3. Select **`STEP (*.step, *.stp)`** and browse to:
   `D:\M.eng\FiniteElementMethode\FEA_Leaf_Spring\resource\Leaf Spring.stp`
4. In the **FE Model** tab:
   - Assign Material: `55Cr3` ($E = 206,000\text{ MPa}$, $\nu = 0.29$, $\rho = 7.85 \times 10^{-9}\text{ tonne/mm}^3$).
   - Boundary Conditions: Pinned at front eye, Roller/Shackle at rear eye.
   - External Load: Center vertical force $F = -8,240.4\text{ N}$ (Static) or $-16,480.8\text{ N}$ (Bump).
5. Right-click **`Analysis-1`** $\rightarrow$ select **`Run`**.
6. Under **`Results`**, view **Von Mises Stress** and **Displacement**.

---

## 📑 Project Report

The complete academic report strictly follows the 5 requested sections:
1. **Introduction**
2. **Problem statement** (Assumptions, Free body diagrams, Force calculations)
3. **Description of finite element model** (Model info, Units, Material properties, Boundary conditions, Mesh info)
4. **Analysis** (Static vertical, 2.0G bump, Braking, Cornering, Strain energy convergence)
5. **Result and Discussion** (Study results summary table, Discussion, Model Revision comparison)

👉 Please read the complete report in: **[`results/result.md`](results/result.md)**.
