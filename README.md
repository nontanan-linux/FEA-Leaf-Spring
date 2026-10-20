# Automotive Suspension Leaf Spring Finite Element Analysis (FEA)

**Academic Course:** Finite Element Method in Engineering (Master of Engineering Program)  
**Instructor:** Asst. Prof. Dr. Monsak Pimsarn  
**Institution:** Faculty of Engineering, King Mongkut's Institute of Technology Ladkrabang (KMITL)  
**Repository:** [https://github.com/nontanan-linux/FEA-Leaf-Spring.git](https://github.com/nontanan-linux/FEA-Leaf-Spring.git)  

---

## 1. Project Overview & Scope

This project presents an object-oriented Finite Element Method (FEM) simulation framework for automotive semi-elliptic leaf springs utilized in light commercial vehicles (LCVs). The platform couples rigid-body vehicle dynamics, 2D/3D curved beam/continuum elastostatics, configuration-driven domain discretization, adaptive mesh refinement, and standardized multi-field post-processing recovery conforming to commercial solver conventions (ANSYS Workbench Mechanical / CalculiX).

### Core Methodological Pillars:
1. **Dynamic Load Derivation:** Analytical calculation of static laden normal forces, dynamic road bump impacts, braking load transfers with axle wind-up torque, and cornering lateral load transfers.
2. **Object-Oriented FEM Architecture:** Decoupled modular pipeline structured into 5 sequential stages (Preprocessing, Analytical Formulation, System Solver, Post-Processing, and Refinement).
3. **Configuration-Driven Execution:** Declarative YAML parameters (`config/*.yaml`) decoupling physical constants, material properties, mesh controls, and operating conditions from source code.
4. **Adaptive Discretization & Verification:** Curvature-sensitive mesh generation with Rivara longest-edge bisection and asymptotic strain energy convergence evaluation.
5. **Structural Design Optimization:** Geometric redesign from constant-thickness section into parabolic thickness tapering to eliminate localized stress singularities.

---

## 2. Finite Element System Architecture

The analysis architecture follows a sequential pipeline where each stage is encapsulated in a dedicated Python class and orchestrated by a master pipeline runner:

```mermaid
graph TD
    CFG["Configuration Layer<br/><b>config/*.yaml</b><br/>(vehicle, solver, model, material)"] --> CL["ConfigManager<br/>(scripts/config_loader.py)"]
    
    CL --> S1["Step 1: Domain Discretization<br/><b>generate_mesh.py</b><br/>(CAD Geometry & Curved Mesh Generation)"]
    CL --> S2["Step 2: Analytical & Assembly<br/><b>leaf_spring_analytical.py</b><br/>(Class LeafSpringAnalytical)"]
    CL --> S3["Step 3: Boundary Value Solver<br/><b>fem_solve.py</b><br/>(Class LeafSpringFEMSolver)"]
    CL --> S4["Step 4: Solution Post-Processing<br/><b>postprocessing.py</b><br/>(Class LeafSpringPostProcessor)"]
    CL --> S5["Step 5: Mesh Convergence<br/><b>refine.py</b><br/>(Class MeshConvergenceStudy)"]
    
    S1 --> PIPE["Master Pipeline Orchestrator<br/><b>leaf_spring_fem.py</b><br/>(Class LeafSpringFEMPipeline)"]
    S2 --> PIPE
    S3 --> PIPE
    S4 --> PIPE
    S5 --> PIPE
    
    PIPE --> R1["results/ansys_workbench_solutions.png<br/>(8-Panel FEA Solution Dashboard)"]
    PIPE --> R2["results/mesh_convergence_study.png<br/>(Dual-Axis Convergence Verification)"]
    PIPE --> R3["results/model_revision_comparison.png<br/>(Baseline vs. Parabolic Tapered Optimization)"]
```

### Module Responsibilities:
- **`config_loader.py` (`ConfigManager`):** Central singleton responsible for discovering, safely type-casting, and caching YAML parameters with engineering fallback defaults.
- **`generate_mesh.py`:** Preprocessing engine that ingests CAD solid/surface geometry (`.stp`, `.stl`, `.inp`), generates 1D/2D/3D discretizations along the camber arc, and reports mesh quality metrics (Jacobian, aspect ratio).
- **`leaf_spring_analytical.py` (`LeafSpringAnalytical`):** Formulation of vehicle dynamics load transfers, classical beam bending equations, local Euler-Bernoulli element stiffness matrices $[k^e]$, rotation transformation $[T]$, and global stiffness assembly $[K_{\text{global}}]$.
- **`fem_solve.py` (`LeafSpringFEMSolver`):** Boundary condition partitioning (pinned front eye, shackle roller rear eye), reduced linear system solution $[K_{ff}]\{U_f\} = \{F_f\}$, reaction force recovery, and strain energy evaluation.
- **`postprocessing.py` (`LeafSpringPostProcessor`):** Stress and strain recovery for all 7 standard FEA field quantities, Factor of Safety evaluation, and publication figure compositing.
- **`refine.py` (`MeshConvergenceStudy`):** Multi-stage mesh convergence verification and Rivara longest-edge bisection adaptive refinement.
- **`leaf_spring_fem.py` (`LeafSpringFEMPipeline`):** Master pipeline coordinator executing the complete simulation sequence via unified API and CLI flags.

---

## 3. Theoretical Framework & Governing Formulations

### 3.1 Vehicle Dynamics & Mechanical Loading
Operational loading conditions are derived from rigid-body vehicle dynamics parameters specified in `config/vehicle_information.yaml`:

- **Static Normal Ride Load (1.0G):**
  $$F_{v,\text{static}} = \frac{M \cdot g \cdot \beta}{2}$$
  where $M$ is Gross Vehicle Weight (GVW), $g$ is gravitational acceleration, and $\beta$ is static rear-axle load fraction ($60\%$).

- **Peak Dynamic Bump Impact ($K_{\text{bump}} \cdot G$):**
  $$F_{v,\text{bump}} = K_{\text{bump}} \cdot F_{v,\text{static}}$$
  accounting for shock acceleration when traversing road irregularities at velocity.

- **Emergency Braking Load Transfer ($a_x = 0.8G$ Deceleration):**
  $$\Delta W_{\text{brake}} = \frac{M \cdot a_x \cdot h_{\text{cg}}}{L_{\text{wb}}}$$
  $$F_{\text{brake}} = \mu \cdot \left(\frac{F_{z,\text{rear}} - \Delta W_{\text{brake}}}{2}\right)$$
  inducing combined vertical load and longitudinal axle wind-up reaction moments.

- **Steady-State Cornering Load Transfer ($a_y = 0.7G$ Lateral Accel):**
  $$\Delta W_{\text{corner}} = \frac{F_{z,\text{rear}} \cdot a_y \cdot h_{\text{cg}}}{g \cdot T_{\text{track}}}$$
  creating asymmetric vertical load distribution between outer and inner leaf springs.

---

### 3.2 Finite Element Formulation & Coordinate Transformation

#### Initial Camber Discretization:
The longitudinal domain $x \in [-L_{\text{span}}/2, +L_{\text{span}}/2]$ is discretized with nodes lying along the free camber profile:
$$y(x) = h_{\text{camber}} \left[1 - \left(\frac{2x}{L_{\text{span}}}\right)^2\right]$$

#### Local Element Stiffness Matrix ($[k_{\text{local}}]_{6 \times 6}$):
For a 2-node cubic Hermite beam element with axial and flexural capabilities (3 DOFs per node: $u_x, u_y, \theta_z$):

$$[k_{\text{local}}] = \begin{bmatrix}
\frac{EA}{L_e} & 0 & 0 & -\frac{EA}{L_e} & 0 & 0 \\
0 & \frac{12EI}{L_e^3} & \frac{6EI}{L_e^2} & 0 & -\frac{12EI}{L_e^3} & \frac{6EI}{L_e^2} \\
0 & \frac{6EI}{L_e^2} & \frac{4EI}{L_e} & 0 & -\frac{6EI}{L_e^2} & \frac{2EI}{L_e} \\
-\frac{EA}{L_e} & 0 & 0 & \frac{EA}{L_e} & 0 & 0 \\
0 & -\frac{12EI}{L_e^3} & -\frac{6EI}{L_e^2} & 0 & \frac{12EI}{L_e^3} & -\frac{6EI}{L_e^2} \\
0 & \frac{6EI}{L_e^2} & \frac{2EI}{L_e} & 0 & -\frac{6EI}{L_e^2} & \frac{4EI}{L_e}
\end{bmatrix}$$

where $L_e$ is element length, $A = b \cdot t$ is cross-sectional area, and $I = \frac{b t^3}{12}$ is second moment of area.

#### Direction-Cosine Transformation Matrix ($[T]_{6 \times 6}$):
Transforming from element local coordinates to global structural coordinates:
$$[T] = \begin{bmatrix}
c & s & 0 & 0 & 0 & 0 \\
-s & c & 0 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 & 0 \\
0 & 0 & 0 & c & s & 0 \\
0 & 0 & 0 & -s & c & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}, \quad c = \frac{dx}{L_e}, \quad s = \frac{dy}{L_e}$$

$$[k_{\text{global}}^e] = [T]^T [k_{\text{local}}] [T]$$
$$[K_{\text{global}}] = \sum_{e=1}^{N_{\text{elem}}} [k_{\text{global}}^e]$$

---

### 3.3 Boundary Enforcement & Linear Equilibrium Solution

Essential kinematic constraints are partitioned into unconstrained ($f$) and prescribed ($c$) degrees of freedom:
- **Front Eye ($x = -L/2$, Node 0):** Pinned pivot constraint ($u_x = 0, u_y = 0, \theta_z \in \mathbb{R}$).
- **Rear Eye ($x = +L/2$, Node $N$):** Swinging shackle roller constraint ($u_y = 0, u_x \in \mathbb{R}, \theta_z \in \mathbb{R}$).

$$\begin{bmatrix} [K_{ff}] & [K_{fc}] \\ [K_{cf}] & [K_{cc}] \end{bmatrix} \begin{Bmatrix} \{U_f\} \\ \{U_c\} \end{Bmatrix} = \begin{Bmatrix} \{F_f\} \\ \{R_c\} \end{Bmatrix}$$

Since prescribed displacements $\{U_c\} = \{0\}$, the equilibrium displacement vector is resolved via direct linear solution:
$$\{U_f\} = [K_{ff}]^{-1} \{F_f\}$$
$$\{R_c\} = [K_{cf}] \{U_f\} - \{F_c\}$$

---

### 3.4 7 Fundamental ANSYS-Compatible Structural Solution Fields

1. **Total Deformation ($U_{\text{total}}$):**
   $$U_{\text{total}} = \sqrt{u_x^2 + u_y^2} \quad [\text{mm}]$$
   Quantifies resultant nodal translation across all active degrees of freedom.

2. **Directional Deformation in Y-Axis ($U_y$):**
   $$U_y = u_y(x) \quad [\text{mm}]$$
   Represents vertical compliance along the gravity and suspension travel axis.

3. **Equivalent Elastic Strain ($\varepsilon_{\text{eq}}$):**
   $$\varepsilon_{\text{eq}} = \frac{\sigma_{\text{vm}}}{E} \quad [\mu\varepsilon]$$
   Measures octahedral distortion strain relative to the material yield strain limit ($\varepsilon_y = \sigma_y / E$).

4. **Equivalent Stress ($\sigma_{\text{vm}}$ - Von Mises Criterion):**
   $$\sigma_{\text{vm}} = \left|\frac{N_{\text{axial}}}{A}\right| + \frac{\max(|M_1|, |M_2|)}{Z} \quad [\text{MPa}]$$
   where $Z = \frac{b t^2}{6}$ is elastic section modulus.

5. **Maximum Principal Normal Stress ($\sigma_1$):**
   $$\sigma_1 = \max\left(0, \sigma_{\text{axial}} + \sigma_{\text{bending}}, -\sigma_{\text{axial}} + \sigma_{\text{bending}}\right) \quad [\text{MPa}]$$
   Identifies peak tensile stress on the convex underside driving high-cycle fatigue microcrack initiation.

6. **Maximum Shear Stress ($\tau_{\max}$ - Tresca Criterion):**
   $$\tau_{\max} = \frac{\sigma_1 - \sigma_3}{2} = \frac{\sigma_1}{2} \quad [\text{MPa}]$$
   Governs yielding slip planes at $45^\circ$ to the longitudinal axis.

7. **Total Elastic Strain Energy ($U_{\text{strain}}$):**
   $$U_{\text{strain}} = \frac{1}{2} \{U\}^T [K_{\text{global}}] \{U\} \quad [\text{N}\cdot\text{m} \text{ / Joules}]$$
   Measures integrated mechanical energy absorbed during spring deflection.

---

### 3.5 Discretization & Refinement Methodology

#### Adaptive Mesh Refinement (Rivara Longest-Edge Bisection):
Curvature-sensitive division bisects any triangular surface element whose longest edge exceeds the threshold $h_{\max}$:
$$\text{If } \max(\|v_1 - v_0\|, \|v_2 - v_1\|, \|v_0 - v_2\|) > h_{\max} \implies \text{Bisect longest edge at midpoint } m$$
Non-conforming nodes are eliminated by recursively bisecting adjacent sharing elements, preserving angle aspect ratios and avoiding degenerate geometries.

#### Mesh Convergence Evaluation Principle:
Discretization independence is verified across successive mesh refinements by monitoring the relative change in integrated strain energy ($\Delta U_h$) and peak Von Mises stress ($\Delta \sigma_h$):
$$\text{Relative Error: } e_{h} = \left|\frac{\Phi_{h} - \Phi_{h-1}}{\Phi_h}\right| \times 100\% < \epsilon_{\text{target}}$$
An asymptotic plateau indicates grid independence where numerical discretization error becomes negligible.

---

### 3.6 Structural Design Optimization Formulation

#### Parabolic Thickness Tapering Profile:
In constant-thickness leaf springs ($t = \text{const}$), bending moment $M(x)$ decreases linearly toward the eyes, causing understressed end regions and sharp stress concentration at the central axle clamp. The optimized parabolic profile tailors thickness to bending moment:
$$t(x) = t_{\text{center}} - (t_{\text{center}} - t_{\text{eye}}) \left(\frac{2x}{L_{\text{span}}}\right)^2$$
- $t_{\text{center}}$: Maximum thickness at axle clamping zone to resist peak bending moments.
- $t_{\text{eye}}$: Reduced thickness at eye ends to maximize elastic compliance and reduce unsprung mass.

---

## 4. Repository Structure

```
├── config/                     # Central YAML Configuration Layer
│   ├── vehicle_information.yaml# Vehicle masses, dimensions, dynamic operating cases
│   ├── solver.yaml             # Boundary conditions, solver settings, unit systems
│   ├── model.yaml              # CAD geometry parameters, mesh convergence stages
│   └── material.yaml           # High-strength spring steel (55Cr3 / SUP9) database
├── resource/                   # 3D CAD Models & Preprocessed Mesh Assets
│   ├── Leaf Spring.stp         # 3D STEP master solid model
│   ├── Leaf Spring.igs         # IGES surface model
│   ├── Leaf Spring.stl         # High-resolution surface triangulation
│   ├── leaf_spring_mesh.inp    # Preprocessed CalculiX / Abaqus input deck
│   └── leaf_spring_mesh.stl    # Refined surface triangulation mesh
├── results/                    # Generated Outputs, Artifacts & Reports
│   ├── result.md               # Complete academic FEA technical report
│   ├── ansys_workbench_solutions.png # 8-Panel ANSYS FEA Solution Dashboard
│   ├── mesh_convergence_study.png    # Dual-Axis Mesh Convergence Verification
│   ├── model_revision_comparison.png # Baseline vs. Parabolic Tapered Comparison
│   ├── vehicle_dynamics_fbd.png      # Vehicle Free Body Diagram
│   └── fixed_support_model.png       # CAD 3D Boundary Restraint Visualization
├── scripts/                    # Object-Oriented Finite Element Method Source Code
│   ├── config_loader.py            # Central Configuration Manager (ConfigManager)
│   ├── generate_mesh.py            # Step 1: CAD import & domain discretization
│   ├── leaf_spring_analytical.py   # Step 2: Class LeafSpringAnalytical
│   ├── fem_solve.py                # Step 3: Class LeafSpringFEMSolver
│   ├── postprocessing.py           # Step 4: Class LeafSpringPostProcessor
│   ├── refine.py                   # Step 5: Class MeshConvergenceStudy
│   ├── leaf_spring_fem.py          # Master Orchestrator: Class LeafSpringFEMPipeline
│   ├── plot_fea_solutions_3d.py    # 3D Open3D field visualizer across 7 solution fields
│   ├── plot_vehicle_dynamics.py    # Vehicle dynamics schematic generator
│   ├── plot_cad_fixed_support.py   # Fixed support renderer
│   ├── plot_cad_mesh.py            # 3D CAD mesh renderer
│   └── plot_mesh_model.py          # Mesh and boundary condition visualizer
├── pyproject.toml              # Build system & metadata configuration
└── requirements.txt            # Python package dependencies
```

---

## 5. Installation & Verification

### 5.1 Clone Repository
```powershell
git clone https://github.com/nontanan-linux/FEA-Leaf-Spring.git
cd FEA-Leaf-Spring
```

### 5.2 Virtual Environment Setup
- **Windows (PowerShell):**
  ```powershell
  python -m venv .venv
  .\.venv\Scripts\Activate.ps1
  ```
- **Linux / macOS:**
  ```bash
  python3 -m venv .venv
  source .venv/bin/activate
  ```

### 5.3 Install Dependencies
```powershell
python -m pip install --upgrade pip
pip install -r requirements.txt
```

### 5.4 Environment Verification
```powershell
python -c "import numpy, scipy, matplotlib, open3d, yaml, trimesh; print('>>> [SUCCESS] All dependencies loaded successfully!')"
```

---

## 6. Execution Guide & Programmatic API

### 6.1 Master Pipeline Runner (`leaf_spring_fem.py`)
Executes the full sequential FEM workflow:
```powershell
# Run the complete analysis pipeline:
python scripts/leaf_spring_fem.py

# Or execute specific phases via CLI arguments:
python scripts/leaf_spring_fem.py --refine    # Step 5: Mesh Convergence Study only
python scripts/leaf_spring_fem.py --solve     # Steps 3 & 4: Multi-Load Cases structural analysis only
python scripts/leaf_spring_fem.py --revision  # Model Revision (Uniform vs. Parabolic) only
python scripts/leaf_spring_fem.py --elements 80 # Custom discretization element count
```

---

### 6.2 Modular Step-by-Step Programmatic Usage

Each module can be instantiated and executed independently in Python:

#### **Step 1: Domain Discretization (`generate_mesh.py`)**
```powershell
python scripts/generate_mesh.py
```

#### **Step 2: Analytical Mechanics & Element Assembly (`leaf_spring_analytical.py`)**
```python
from leaf_spring_analytical import LeafSpringAnalytical

analytical = LeafSpringAnalytical() # Automatically reads config/*.yaml
loads = analytical.calculate_vehicle_loads()
mechanics = analytical.calculate_leaf_spring_mechanics(loads)
```

#### **Step 3: Boundary Value Problem Solver (`fem_solve.py`)**
```python
from fem_solve import LeafSpringFEMSolver

solver = LeafSpringFEMSolver()
raw_result = solver.solve(num_elements=60, F_vertical=-8240.40)
```

#### **Step 4: Post-Processing & Solution Fields Recovery (`postprocessing.py`)**
```python
from postprocessing import LeafSpringPostProcessor

post = LeafSpringPostProcessor()
result = post.recover_element_solutions(raw_result)
post.print_solution_summary(result, "Static Normal Ride")
post.plot_ansys_workbench_solutions(result)
```

#### **Step 5: Adaptive Refinement & Mesh Convergence (`refine.py`)**
```python
from refine import MeshConvergenceStudy

study = MeshConvergenceStudy()
stages = study.run_study()
```

#### **Master Pipeline Class (`leaf_spring_fem.py`)**
```python
from leaf_spring_fem import LeafSpringFEMPipeline

pipeline = LeafSpringFEMPipeline()
pipeline.run_all(num_elements=60)
```

---

## 7. Numerical Results & Technical Report Reference

All quantitative finite element simulation results, multi-load case numerical evaluations, convergence tables, and structural optimization comparisons are detailed in the project technical report:

👉 **[`results/result.md`](results/result.md)**

### Key Visual Output Artifacts Generated in `results/`:
- **[`results/ansys_workbench_solutions.png`](results/ansys_workbench_solutions.png):** 8-Panel executive dashboard presenting all 7 primary solution fields.
- **[`results/mesh_convergence_study.png`](results/mesh_convergence_study.png):** Dual-axis convergence chart of strain energy and Von Mises stress.
- **[`results/model_revision_comparison.png`](results/model_revision_comparison.png):** Longitudinal stress distribution comparison (Uniform vs. Parabolic Tapered).

---

## 8. Commercial Solver Verification (PrePoMax / CalculiX)

To verify the model in commercial / open-source graphical solvers:
1. Launch **PrePoMax**.
2. Select **`File`** $\rightarrow$ **`Import...`** and load:
   `resource/Leaf Spring.stp`
3. Under the **FE Model** tab:
   - Assign Material: `55Cr3` ($E = 206,000\text{ MPa}$, $\nu = 0.29$, $\rho = 7.85 \times 10^{-9}\text{ tonne/mm}^3$).
   - Boundary Conditions: Pinned constraint at front eye, roller constraint at rear eye.
   - Applied Load: Center vertical force $F_v$ from `config/vehicle_information.yaml`.
4. Run the linear static analysis and verify field contours.

---

## 9. References
1. **SAE HS-788:** *Manual on Design and Application of Leaf Springs*, Society of Automotive Engineers, Warrendale, PA.
2. **JIS G 4801 / DIN EN 10089:** *Spring Steels (SUP9 / 55Cr3 Material Specifications)*.
3. **Cook, R. D., Malkus, D. S., Plesha, M. E., & Witt, R. J. (2001):** *Concepts and Applications of Finite Element Analysis*, 4th Edition, John Wiley & Sons.
4. **Shigley, J. E., & Mischke, C. R. (2020):** *Mechanical Engineering Design*, 11th Metric Edition, McGraw-Hill.
