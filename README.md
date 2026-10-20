# Automotive Leaf Spring FEA Project (Python & PrePoMax)

**Course:** Finite Element Method in Engineering (M.Eng Program)  
**Instructor:** Asst. Prof. Dr. Monsak Pimsarn  
**Project Location:** `D:\M.eng\FiniteElementMethode\FEA_Leaf_Spring`  

---

## 📁 Project Structure

```
D:\M.eng\FiniteElementMethode\FEA_Leaf_Spring\
├── README.md                   # This project guide and execution instructions
├── pyproject.toml              # Project packaging configuration & build metadata
├── requirements.txt            # Python package dependencies and versions
├── config\                     # Project configuration YAML files
│   ├── vehicle_information.yaml# Vehicle specifications, masses, and dynamics
│   ├── solver.yaml             # Solver methods, units, material, BCs, and load cases
│   ├── model.yaml              # CAD geometry, mesh parameters, and model revisions
│   └── material.yaml           # Comprehensive material properties database (55Cr3, Fasteners, Bushings, GFRP)
├── resource\                   # 3D CAD & Geometry files (also aliased as resources\)
│   ├── Leaf Spring.stp         # 3D STEP solid model for CAD/PrePoMax import
│   ├── Leaf Spring.igs         # IGES format
│   ├── Leaf Spring.stl         # 3D STL mesh model
│   ├── Leaf Spring.jpg         # Model visual preview
│   ├── leaf_spring_cad_model.jpg # Rendered CAD solid geometry image
│   ├── leaf_spring_mesh.inp    # 🎯 Preprocessed CalculiX/Abaqus FEA mesh (Generated into resources/)
│   └── leaf_spring_mesh.stl    # 🎯 Refined surface mesh (Generated into resources/)
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
│   ├── leaf_spring_solutions_3d.png # Figure 7: Complete 3D Finite Element Solution Results (7 Field Quantities)
│   ├── solution_1_total_deformation.png # 3D Total Deformation (U_total)
│   ├── solution_2_directional_deformation_y.png # 3D Directional Deformation (U_y)
│   ├── solution_3_equivalent_strain.png # 3D Equivalent Elastic Strain (eps_eq)
│   ├── solution_4_equivalent_stress.png # 3D Equivalent Stress (Von Mises)
│   ├── solution_5_max_principal_stress.png # 3D Maximum Principal Stress (sigma_1)
│   ├── solution_6_max_shear_stress.png # 3D Maximum Shear Stress (tau_max)
│   ├── solution_7_strain_energy.png # 3D Strain Energy Distribution (U_strain)
│   └── model_revision_comparison.png # Figure 8: Baseline vs. revised comparison chart
└── scripts\                    # Python source code for analysis & FEA
    ├── generate_mesh.py            # Step 1: CAD import & domain discretization (mesh generation)
    ├── leaf_spring_analytical.py   # Step 2: Analytical mechanics, element equations & global stiffness assembly
    ├── fem_solve.py                # Step 3: Boundary condition enforcement & linear system solver
    ├── postprocessing.py           # Step 4: Stress/strain field recovery & ANSYS visualization dashboards
    ├── refine.py                   # Step 5: Adaptive refinement (Rivara bisection) & mesh convergence study
    ├── leaf_spring_fem.py          # 🎯 Master Orchestrator: Executes full FEM pipeline (Steps 1 to 5)
    ├── plot_fea_solutions_3d.py    # 3D Open3D FEA visualizer for all 7 primary solution fields
    ├── plot_vehicle_dynamics.py    # Vehicle dynamics Free Body Diagram generator (Figure 1)
    ├── plot_cad_fixed_support.py   # Direct CAD 3D renderer (Open3D) & Fixed Support hole highlighter
    ├── plot_cad_mesh.py            # Direct CAD 3D FEA mesh renderer with Rivara bisection refinement
    ├── plot_mesh_model.py          # 3D STL mesh and boundary condition generator
    └── plot_fixed_support.py       # Dedicated 3D Fixed Support visualizer
```

---

## ⚙️ Installation & Project Setup

### 1. Prerequisites
- **Python**: Version `3.10` or higher (Tested and verified on `Python 3.12.6`).
- **Operating System**: Windows 10 / 11, Linux (Ubuntu), or macOS.
- **Git**: For version control and cloning repository.
- *(Optional)* **PrePoMax (v2.0.0+) / CalculiX**: For graphical FEA inspection and native solver export.

### 2. Install from Git

You can set up the project using either **Option A (Clone & Local Workspace)** or **Option B (Direct Pip Install from Git)**:

#### Option A: Clone Repository from Git (Recommended for Running Scripts & Viewing Visualizations)
```powershell
# Clone the repository
git clone https://github.com/nontanan-linux/FEA-Leaf-Spring.git

# Navigate into the project directory
cd FEA-Leaf-Spring
```

#### Option B: Direct Python Package Install from Git
Install directly into your active Python environment via `pip`:
```powershell
pip install git+https://github.com/nontanan-linux/FEA-Leaf-Spring.git
```
*(Or in editable development mode from within the local workspace: `pip install -e .`)*

### 3. Create & Activate Virtual Environment (Recommended)
Setting up an isolated virtual environment prevents package conflicts:

- **Windows (PowerShell):**
  ```powershell
  python -m venv .venv
  .\.venv\Scripts\Activate.ps1
  ```
  > **Note on PowerShell Execution Policy:**  
  > If script execution is blocked, enable it for the current process:  
  > `Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass`

- **Linux / macOS:**
  ```bash
  python3 -m venv .venv
  source .venv/bin/activate
  ```

### 4. Install Dependencies
Install all required numerical, CAD processing, and 3D visualization packages via `requirements.txt`:
```powershell
python -m pip install --upgrade pip
pip install -r requirements.txt
```

#### Core Package Reference:
| Package | Minimum Version | Purpose in Project |
| :--- | :--- | :--- |
| **`numpy`** | $\ge 1.24.0$ | Matrix assembly, global stiffness matrix operations, vector mechanics |
| **`scipy`** | $\ge 1.10.0$ | Sparse linear algebra solver (`spsolve`), numerical integration |
| **`matplotlib`** | $\ge 3.7.0$ | Publication-grade charts, FBD schematics, convergence plots, report figures |
| **`open3d`** | $\ge 0.17.0$ | 3D CAD mesh rendering, raycasting, vertex coloring, and 3D contour field generation |
| **`PyYAML`** | $\ge 6.0$ | Structured configuration parsing (`config/*.yaml`) |
| **`pillow`** | $\ge 10.0.0$ | High-resolution image post-processing and figure compositing |
| **`trimesh`** | $\ge 4.0.0$ | 3D surface mesh manipulation and boundary condition geometry queries |
| **`gmsh`** | $\ge 4.12.0$ | Direct 3D CAD (STEP/IGES) parametric solid/surface meshing and discretization |

### 5. Verify Installation
Verify that all dependencies and visualization engines are loaded correctly:
```powershell
python -c "import numpy, scipy, matplotlib, open3d, yaml, trimesh; print('>>> [SUCCESS] All FEA dependencies installed successfully!')"
```

---

## 🚀 How to Run the Python Scripts

### 🎯 Master Orchestrator: Full FEM Pipeline (`leaf_spring_fem.py`)
Executes the end-to-end Finite Element Method workflow orchestrating all 5 modules sequentially:
```powershell
# Run the complete analysis (Mesh Convergence -> Multi-Load Cases -> Model Revision):
python scripts\leaf_spring_fem.py

# Or run specific stages via CLI flags:
python scripts\leaf_spring_fem.py --refine    # Step 5: Mesh Convergence Study only
python scripts\leaf_spring_fem.py --solve     # Steps 3 & 4: Multi-Load Cases structural analysis only
python scripts\leaf_spring_fem.py --revision  # Model Revision (Uniform vs. Parabolic) only
```

---

### 🧩 Modular FEM Steps (Sequential Execution)

The core FEA solver is decomposed into 5 decoupled, independently executable modules matching the canonical steps of the Finite Element Method:

#### **Step 1: CAD Import & Domain Discretization (`generate_mesh.py`)**
Imports 3D CAD (`.stp` / `.step`) or surface meshes (`.stl`), generates curved beam/frame or shell/solid discretizations, and calculates mesh quality metrics (Jacobian, aspect ratio):
```powershell
python scripts\generate_mesh.py
```

#### **Step 2: Analytical Formulations, Element Equations & Assembly (`leaf_spring_analytical.py`)**
Derives vehicle dynamic load transfers (1G static, 2G bump, 0.8G braking, 0.7G cornering), defines local beam element stiffness $[k^e]$, rotation transformation $[T]$, global assembly $[K_{\text{global}}]$, and boundary DOFs:
```powershell
python scripts\leaf_spring_analytical.py
```

#### **Step 3: Boundary Enforcement & Linear Equation Solver (`fem_solve.py`)**
Partitions boundary conditions (pinned front eye, shackle roller rear eye), solves $[K_{\text{free}}] \{U_{\text{free}}\} = \{F_{\text{free}}\}$, and recovers reaction forces and strain energy:
```powershell
python scripts\fem_solve.py
```

#### **Step 4: Post-Processing & Solution Fields (`postprocessing.py`)**
Recovers the 7 standard ANSYS-compatible FEA solution fields ($\sigma_{\text{vm}}$, $\sigma_1$, $\tau_{\max}$, $\varepsilon_{\text{eq}}$, $U_{\text{total}}$, $U_y$, $U_{\text{strain}}$), safety factors, and generates the executive 8-panel dashboard:
```powershell
python scripts\postprocessing.py
```
*Generated output:* `results/ansys_workbench_solutions.png`

#### **Step 5: Adaptive Mesh Refinement & Convergence (`refine.py`)**
Performs Rivara longest-edge bisection adaptive refinement and 6-stage 3D mesh convergence study (3,124 to 29,172 elements) with non-overlapping callout charts:
```powershell
python scripts\refine.py
```
*Generated output:* `results/mesh_convergence_study.png`

### 5. Generate Vehicle Dynamics Diagram & CAD Mesh Visualizations
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
