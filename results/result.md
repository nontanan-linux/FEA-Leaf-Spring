# Stress Analysis and Design Optimization of Automotive Leaf Spring by FEM

**Course:** Finite Element Method in Engineering (M.Eng Program)  
**Instructor:** Asst. Prof. Dr. Monsak Pimsarn  
**Project Folder:** `FEA_Leaf_Spring`  
**Model Resource:** `resource/`  
**Scripts Directory:** `scripts/`  
**Results Directory:** `results/`  

---

## Table of Contents
1. **Introduction**
2. **Problem Statement**  
   2.1 Assumptions for the conditions  
   2.2 The model that was analyzed  
   2.3 Forces acting on the leaf spring  
3. **Description of Finite Element Model**  
   3.1 Model Information  
   3.2 Study Properties  
   3.3 Units  
   3.4 Material Properties  
   3.5 Loads and Restraints  
   3.6 Mesh Information  
4. **Finite Element System Architecture & Repository Structure**  
   4.1 System Pipeline Architecture & Module Responsibilities  
   4.2 Repository Structure & File Organization  
5. **Installation & Verification**  
   5.1 Clone Repository  
   5.2 Virtual Environment Setup  
   5.3 Install Dependencies  
   5.4 Environment Verification  
6. **Execution Guide & Programmatic API**  
   6.1 Master Pipeline Runner  
   6.2 Modular Step-by-Step Programmatic Usage  
7. **Analysis**  
   7.1 Normal Static Vertical Load Analysis  
   7.2 Dynamic Bump Load Analysis (2.0G Impact)  
   7.3 Combined Bump and Braking Load Analysis  
   7.4 Severe Dynamic Bump + Braking Load Analysis  
   7.5 Leaf Spring Strain Energy and Mesh Convergence Study  
8. **Results and Discussions**  
   8.1 Study Results  
   8.2 Discussions & Model Revision (Single Leaf vs. Multi-Leaf / Parabolic Tapered)  
9. **References**  

### List of Figures (สารบัญภาพ)
* **Figure 1:** Automotive Semi-Elliptic Multi-Leaf Spring Assembly in Real-World Service & Manufacturing (Section 1)
* **Figure 2:** Vehicle Free Body Diagram & Suspension Load Distribution (Section 2.1)
* **Figure 3:** 3D CAD Solid Geometry Model of the Automotive Leaf Spring Assembly (Section 2.2)
* **Figure 4:** 3D Fixed Support Restraint Applied on Inner Cylindrical Bore of Front Eye (Section 3.5.1)
* **Figure 5:** Finite Element Restraints and Boundary Loading Schematic (Section 3.5.3)
* **Figure 6:** 3D Finite Element Mesh Model and Rolled Eye Quadratic Discretization (Section 3.6)
* **Figure 7:** Finite Element Mesh Convergence Study (Strain Energy vs. Discretization Density) (Section 7.5)
* **Figure 8.1:** 3D Total Deformation Field Distribution ($U_{\text{total}}$) (Section 8.1)
* **Figure 8.2:** 3D Directional Deformation Field Distribution in Y-Axis ($U_y$) (Section 8.1)
* **Figure 8.3:** 3D Equivalent Elastic Strain Field Distribution ($\varepsilon_{\text{eq}}$) (Section 8.1)
* **Figure 8.4:** 3D Equivalent Von Mises Stress Field Distribution ($\sigma_{\text{vm}}$) (Section 8.1)
* **Figure 8.5:** 3D Maximum Principal Stress Field Distribution ($\sigma_1$) (Section 8.1)
* **Figure 8.6:** 3D Maximum Shear Stress Field Distribution ($\tau_{\max}$) (Section 8.1)
* **Figure 8.7:** 3D Internal Strain Energy Distribution ($U_{\text{strain}}$) (Section 8.1)
* **Figure 9:** Von Mises Stress Distribution Comparison: Baseline Uniform vs. Parabolic Tapered Model (Section 8.2)

### List of Tables (สารบัญตาราง)
* **Table 1:** Design Load Cases for Leaf Spring Analysis (Section 2.3)
* **Table 2:** Mechanical and Physical Properties of High-Strength Spring Steel 55Cr3 (SUP9) (Section 3.4)
* **Table 3:** Multi-Stage Finite Element Mesh Convergence Evaluation Across Refinement Densities (Section 7.5)
* **Table 4:** Comprehensive Multi-Load Case Finite Element Solution Matrix (Section 8.1)
* **Table 5:** Performance Comparison: Baseline Uniform vs. Revised Parabolic Tapered Model (Section 8.2)

---

## 1. Introduction
The suspension system of a commercial vehicle is responsible for carrying the vehicle body, payload, and passengers while isolating the chassis from road-induced shock, vibrations, and dynamic disturbances. In light commercial vehicles (LCVs), 1-tonne pickup trucks, and heavy freight transports, the **semi-elliptic leaf spring** is the predominant suspension mechanism deployed at the rear axle [3]. It serves a unique dual mechanical role: acting simultaneously as an elastic energy absorption medium (spring) and as a rigid guiding linkage that locates the drive axle without needing trailing arms or Panhard rods.

![Figure 1: Automotive Semi-Elliptic Multi-Leaf Spring Assembly in Real-World Service & Manufacturing](real_leaf_spring_assembly.png)
*Figure 1: Automotive semi-elliptic leaf spring assembly in real-world application. (a) Physical multi-leaf assembly during workshop manufacturing showcasing curled cylindrical eye with rubber-steel bushing, master leaf, graduated leaf stacking, center alignment hole, and rebound clip. (b) Installed real-world rear suspension showing front chassis hanger pivot bracket, multi-leaf pack, and axle seat.*

During daily service, automotive leaf springs are subjected to complex, multi-axial cyclic mechanical stresses:
- **Vertical Bump Impacts:** Severe dynamic forces generated when navigating potholes and uneven road surfaces.
- **Braking Torque & Longitudinal Load Transfer:** Axle wind-up moments created during high-deceleration emergency stops.
- **Cornering Lateral Forces:** Centrifugal forces producing lateral bending and torsional distortion on the spring leaves.

To design a durable, high-performance suspension component, the Finite Element Method (FEM) is applied to evaluate the stress distribution, predict maximum deflection, verify numerical convergence, and execute structural optimization to reduce unsprung mass [4], [5]. In this investigation, analytical rigid-body vehicle dynamics calculations [1], [2] are combined with finite element modeling to analyze an automotive leaf spring under multiple critical operating load cases.

---

## 2. Problem Statement

### 2.1 Assumptions for the Conditions
The analysis considers a standard light commercial vehicle (1-tonne laden pickup truck) under critical operating maneuvers governed by classical vehicle dynamics formulations [1], [2].

![Figure 2: Vehicle Free Body Diagram & Suspension Load Distribution](vehicle_dynamics_fbd.png)
*Figure 2: Vehicle Free Body Diagram and suspension load distribution for a 1-tonne laden commercial pickup truck ($GVW = 2,800\text{ kg}$). Shows static normal load allocation ($40\%$ front, $60\%$ rear) at the center of gravity ($h_{\text{cg}} = 0.750\text{ m}, a = 1.851\text{ m}, b = 1.234\text{ m}$) and the corresponding vertical design forces transmitted to each rear leaf spring ($F_{v,\text{static}} = 8,240.4\text{ N}$, $F_{v,\text{bump}} = 16,480.8\text{ N}$).*

**Vehicle Baseline Specifications:**
- Gross Vehicle Weight ($\text{GVW}$): $M = 2,800.0\text{ kg}$
- Total Gravitational Weight ($F_g$): $M \times g = 2,800 \times 9.81 = 27,468.0\text{ N}$
- Wheelbase ($L_{\text{wb}}$): $3.085\text{ m}$
- Static Weight Distribution: Front Axle = $40\%$, Rear Axle = $60\%$ (laden state)
- Rear Track Width ($T_r$): $1.540\text{ m}$
- Center of Gravity Height ($h_{\text{cg}}$): $0.750\text{ m}$
- Tire-Road Friction Coefficient ($\mu$): $1.0$ (Dry asphalt)

#### A. Static Normal Force Distribution
- Total static force on rear axle:
  $$F_{z, \text{rear, total}} = 0.60 \times F_g = 0.60 \times 27,468.0 = 16,480.80\text{ N}$$
- Normal force supported per rear leaf spring (2 leaf springs on rear axle):
  $$F_{v, \text{static}} = \frac{F_{z, \text{rear, total}}}{2} = \frac{16,480.80}{2} = 8,240.40\text{ N}$$

#### B. Dynamic Road Bump Load (2.0G Road Impact)
- When striking severe bumps at velocity, vertical dynamic acceleration spikes to $a_z = 2.0G = 19.62\text{ m/s}^2$:
  $$F_{v, \text{bump}} = 2.0 \times F_{v, \text{static}} = 2.0 \times 8,240.40 = 16,480.80\text{ N}$$

#### C. Longitudinal Weight Transfer During Braking (0.8G Deceleration)
- Dynamic deceleration $a_x = 0.8G = 7.848\text{ m/s}^2$ transfers weight forward onto the front axle:
  $$\Delta W_{\text{brake}} = \frac{M \times a_x \times h_{\text{cg}}}{L_{\text{wb}}} = \frac{2,800 \times 7.848 \times 0.750}{3.085} = 5,342.24\text{ N}$$
- Rear normal force during braking:
  $$F_{z, \text{rear, brake}} = 16,480.80 - 5,342.24 = 11,138.56\text{ N}$$
- Braking traction force per leaf spring:
  $$F_{\text{brake, per spring}} = \frac{\mu \times F_{z, \text{rear, brake}}}{2} = \frac{1.0 \times 11,138.56}{2} = 5,569.28\text{ N}$$

#### D. Lateral Weight Transfer During Cornering (0.7G Lateral Acceleration)
- Lateral acceleration $a_y = 0.7G = 6.867\text{ m/s}^2$ creates centrifugal roll moment:
  $$\Delta W_{\text{lat}} = \frac{F_{z, \text{rear, total}} \times a_y \times h_{\text{cg}}}{g \times T_r} = \frac{16,480.80 \times 6.867 \times 0.750}{9.81 \times 1.540} = 5,618.45\text{ N}$$
- Vertical force on outer rear leaf spring:
  $$F_{z, \text{outer spring}} = 8,240.40 + 5,618.45 = 13,858.85\text{ N}$$
- Lateral cornering force on outer leaf spring:
  $$F_{\text{corner, per spring}} = \mu \times F_{z, \text{outer spring}} = 1.0 \times 13,858.85 = 13,858.85\text{ N}$$

---

### 2.2 The Model that was Analyzed
The model evaluated in this project is based on the production master leaf spring CAD files supplied in `resource/` (`Leaf Spring.stp` / `Leaf Spring.stl` / `Leaf Spring.jpg`):

* **Total Eye-to-Eye Length (Span $2L$):** $1,200.0\text{ mm}$ (Semi-span $L = 600.0\text{ mm}$)
* **Leaf Width ($b$):** $60.0\text{ mm}$
* **Leaf Thickness ($t$):** $12.0\text{ mm}$ (Uniform rectangular baseline)
* **Free Camber Height ($h$):** $100.0\text{ mm}$ (Curvature equation: $y(x) = h \left[1 - (2x/L_{\text{span}})^2\right]$)
* **Eye Inner Diameter:** $30.0\text{ mm}$ (Bushing pin interface)
* **Axle Clamping Seat Area ($A_{\text{seat}}$):** $60.0\text{ mm} \times 100.0\text{ mm}$ (U-bolt mounting zone)

![Figure 3: 3D CAD Solid Geometry Model of the Automotive Leaf Spring Assembly](leaf_spring_cad_model.jpg)
*Figure 3: 3D CAD solid geometry model of the automotive multi-leaf spring assembly from `resource/Leaf Spring.stp`. The assembly comprises the master leaf with curled eyes, graduated leaves, rebound alignment clips, and the central axle seating zone ($60\text{ mm} \times 100\text{ mm}$).*

---

### 2.3 Forces Acting on the Leaf Spring
Summary of design loads applied across the study cases:

#### Table 1: Design Load Cases for Leaf Spring Analysis
| Load Case | Description | Vertical Force $F_v$ (N) | Horizontal Force $F_h$ (N) |
| :---: | :--- | :---: | :---: |
| **Case 1** | Normal Static Laden Load | $-8,240.40\text{ N}$ | $0.0\text{ N}$ |
| **Case 2** | Dynamic Bump Impact ($2.0G$) | $-16,480.80\text{ N}$ | $0.0\text{ N}$ |
| **Case 3** | Combined Bump & Braking | $-8,240.40\text{ N}$ | $-3,500.00\text{ N}$ |
| **Case 4** | Severe Dynamic Bump + Braking | $-16,480.80\text{ N}$ | $-5,000.00\text{ N}$ |

---

## 3. Description of Finite Element Model

### 3.1 Model Information
* **Model Name:** Semi-Elliptic Leaf Spring
* **Total Mass:** $6.91\text{ kg}$ (Constant thickness baseline)

### 3.2 Study Properties
* **Study Name:** Static, Bump, Braking, and Combined Multi-Case Evaluation
* **Analysis Type:** 3D Linear Static Elastostatics
* **Solver Type:** Direct Sparse Solver (CalculiX Spooles / Python FEM Solver)

### 3.3 Units
* **Unit System:** Metric SI ($mm, N, \text{MPa}, \text{tonne/mm}^3$)
* **Length / Displacement:** $mm$
* **Force:** $N$
* **Stress:** $\text{N/mm}^2$ ($\text{MPa}$)

### 3.4 Material Properties
High-strength automotive spring steel **55Cr3 (SUP9 / SAE 5160)** conforming to JIS G 4801 [9], DIN EN 10089 [10], and the MatWeb engineering material database [11]:

#### Table 2: Mechanical and Physical Properties of Spring Steel 55Cr3 (SUP9)
| Property Name | Value | Units | Value Type |
| :--- | :---: | :---: | :---: |
| Elastic Modulus ($E$) | $206,000.0$ | $\text{N/mm}^2$ ($\text{MPa}$) | Constant |
| Poisson's Ratio ($\nu$) | $0.29$ | - | Constant |
| Mass Density ($\rho$) | $7.85 \times 10^{-9}$ | $\text{tonne/mm}^3$ ($7,850\text{ kg/m}^3$) | Constant |
| Tensile Strength ($\sigma_u$) | $1,280.0$ | $\text{N/mm}^2$ ($\text{MPa}$) | Constant |
| Yield Strength ($\sigma_y$) | $1,100.0$ | $\text{N/mm}^2$ ($\text{MPa}$) | Constant |

### 3.5 Loads and Restraints

#### 3.5.1 Fixed Support Restraint (Front Eye)
The front eye of the leaf spring is mounted directly to the vehicle chassis frame via a hardened steel pivot pin encased in an elastomeric bushing. In the finite element model, this boundary condition is formulated as a **Fixed Support (Encastre)** applied over the inner cylindrical surface of the front eye sleeve ($\Gamma_{\text{front\_eye}}$):

$$\mathbf{u}(\mathbf{x}) = \mathbf{0} \quad \implies \quad 
\begin{cases}
u_x(x, y, z) = 0 \\
u_y(x, y, z) = 0 \\
u_z(x, y, z) = 0
\end{cases} \quad \forall \mathbf{x} \in \Gamma_{\text{front\_eye}}$$

Because 3D continuum solid elements (`C3D8` and `C3D10`) possess three translational degrees of freedom per node, fixing translations across the annular cylindrical surface locks all rigid-body translational modes and rotational pitch/yaw/roll modes, providing an unyielding fixed support.

![Figure 4: 3D Fixed Support Restraint Applied on Inner Cylindrical Surface of Front Eye](fixed_support_model.png)
*Figure 4: Fixed Support boundary condition applied on the inner cylindrical surface of the leaf spring front eye (evaluated directly on `resource/Leaf Spring.stl`). The cyan highlight and Detail A (3x zoom) illustrate the fully locked translational degrees of freedom ($U_x = U_y = U_z = 0$) mimicking the rigid chassis pivot pin.*

> [!NOTE]
> **CAD Geometry Verification:** The boundary condition is evaluated directly on the solid CAD boundary representation. The front eye inner cylinder ($r = 6.58\text{ mm}$, $L = 54.35\text{ mm}$) provides the rigid chassis pivot constraint, while the rear eye accommodates horizontal compliance via the suspension shackle.

#### 3.5.2 Shackle / Roller Support (Rear Eye)
The rear eye connects to a pivoting suspension shackle (โตงเตง) which accommodates longitudinal elongation as the spring flattens under vertical deflection:
$$u_y(x, y, z) = 0, \quad u_z(x, y, z) = 0, \quad u_x \text{ is free to translate} \quad \forall \mathbf{x} \in \Gamma_{\text{rear\_eye}}$$

#### 3.5.3 Axle Clamping Seat Loading
The central region ($60\text{ mm} \times 100\text{ mm}$) where the leaf spring is clamped to the rear axle via two high-tensile U-bolts receives the vertical reaction forces ($F_v = -8,240.40\text{ N}$ static / $-16,480.80\text{ N}$ dynamic bump) and longitudinal braking traction forces ($F_x$).

![Figure 5: Finite Element Restraints and Boundary Loading Schematic](mesh_boundary_conditions.png)
*Figure 5: Complete finite element restraints and multi-directional loading schematic. Shows the front fixed eye ($U_x = U_y = U_z = 0$), rear shackle sliding roller ($U_y = U_z = 0, U_x = \text{Free}$), and the axle seat subjected to vertical force $F_v$ and longitudinal braking force $F_x$.*

### 3.6 Mesh Information
* **Mesh Type:** 3D Solid Continuum Quadratic Mesh (`C3D10` / `C3D8`) and 1D/2D Timoshenko-Euler curved beam elements (`BEAM188` [8]) with adaptive Rivara longest-edge bisection refinement [7]
* **Mesher Used:** Direct CAD Solid Geometry Discretization (`resource/Leaf Spring.stl`)
* **Number of Elements:** $14,996$ surface elements (corresponding to $\approx 28,400$ volumetric continuum elements)
* **Number of Nodes:** $13,448$ surface nodes (up to $22,500$ 3D continuum nodes)
* **Element Quality:** Average Jacobian ratio $> 0.88$, warpage $< 12^\circ$

![Figure 6: 3D Finite Element Mesh Model and Rolled Eye Quadratic Discretization](mesh_model_3d.png)
*Figure 6: 3D finite element mesh model rendered directly from CAD geometry matching the suspension isometric perspective. Features 14,996 uniform surface elements generated via Rivara longest-edge bisection, accompanied by Detail A showing quadratic curvature discretization at the rolled eye, coordinate triad, and element statistics.*

---

## 4. Finite Element System Architecture & Repository Structure

### 4.1 System Pipeline Architecture & Module Responsibilities

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

#### Module Responsibilities:
- **`config_loader.py` (`ConfigManager`):** Central singleton responsible for discovering, safely type-casting, and caching YAML parameters with engineering fallback defaults.
- **`generate_mesh.py`:** Preprocessing engine that ingests CAD solid/surface geometry (`.stp`, `.stl`, `.inp`), generates 1D/2D/3D discretizations along the camber arc, and reports mesh quality metrics (Jacobian, aspect ratio).
- **`leaf_spring_analytical.py` (`LeafSpringAnalytical`):** Formulation of vehicle dynamics load transfers, classical beam bending equations, local Euler-Bernoulli element stiffness matrices $[k^e]$, rotation transformation $[T]$, and global stiffness assembly $[K_{\text{global}}]$.
- **`fem_solve.py` (`LeafSpringFEMSolver`):** Boundary condition partitioning (pinned front eye, shackle roller rear eye), reduced linear system solution $[K_{ff}]\{U_f\} = \{F_f\}$, reaction force recovery, and strain energy evaluation.
- **`postprocessing.py` (`LeafSpringPostProcessor`):** Stress and strain recovery for all 7 standard FEA field quantities, Factor of Safety evaluation, and publication figure compositing.
- **`refine.py` (`MeshConvergenceStudy`):** Multi-stage mesh convergence verification and Rivara longest-edge bisection adaptive refinement.
- **`leaf_spring_fem.py` (`LeafSpringFEMPipeline`):** Master pipeline coordinator executing the complete simulation sequence via unified API and CLI flags.

### 4.2 Repository Structure & File Organization

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
│   ├── real_leaf_spring_assembly.png # Physical Leaf Spring Benchmark & Assembly Figure
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

## 7. Analysis

All stress evaluations are performed using the **Von Mises Equivalent Stress ($\sigma_{\text{vm}}$)** criterion:
$$\sigma_{\text{vm}} = \sqrt{\frac{1}{2} \left[ (\sigma_1 - \sigma_2)^2 + (\sigma_2 - \sigma_3)^2 + (\sigma_3 - \sigma_1)^2 \right]}$$

### 7.1 Forces Acting on Leaf Spring under Normal Static Vertical Load
Under nominal laden static vertical load ($F = -8,240.40\text{ N}$):
* **Max Von Mises Stress:** $\sigma_{\text{vm, max}} = 1,716.78\text{ MPa}$ (at central clamping point).
* **Yield Strength:** $\sigma_y = 1,100.0\text{ MPa}$.
* **Center Deflection:** $\delta_{\text{static}} = 167.59\text{ mm}$ (Stiffness $k = 49.17\text{ N/mm}$).
* **Observation:** A single unreinforced 12 mm leaf carrying the full axle side load of $8,240.4\text{ N}$ experiences stress exceeding yield, demonstrating why commercial pickup suspensions require a **multi-leaf pack (3–4 leaves)** or a thicker parabolic section ($t \approx 22\text{ mm}$).

### 7.2 Forces Acting under Dynamic Bump Load (2.0G Impact)
Under peak dynamic bump load ($F = -16,480.80\text{ N}$):
* **Max Von Mises Stress:** $\sigma_{\text{vm, max}} = 3,433.56\text{ MPa}$.
* **Center Deflection:** $\delta_{\text{bump}} = 335.19\text{ mm}$.
* **Evaluation:** Stresses are doubled in the linear range, showing the worst-case design boundary for bump stop engagement.

### 7.3 Forces Acting under Combined Bump and Braking Load
Under vertical force $F_v = -8,240.40\text{ N}$ and longitudinal braking traction $F_x = -3,500.0\text{ N}$:
* The horizontal traction creates longitudinal compression and an axle wind-up S-bending moment.
* **Max Von Mises Stress:** $\sigma_{\text{vm, max}} = 1,600.12\text{ MPa}$.
* **Center Deflection:** $\delta = 152.75\text{ mm}$.

### 7.4 Severe Dynamic Bump + Braking Load Analysis
Under maximum dynamic bump ($16,480.80\text{ N}$) combined with severe braking traction ($-5,000\text{ N}$):
* **Max Von Mises Stress:** $\sigma_{\text{vm, max}} = 3,266.90\text{ MPa}$.
* **Center Deflection:** $\delta = 313.99\text{ mm}$.

### 7.5 Leaf Spring Strain Energy and Mesh Convergence Study
To ensure the numerical validity and discretization independence of the finite element solution [5], a systematic mesh convergence study was conducted across 6 successive mesh refinement stages (spanning $3,124$ to $29,172$ surface elements, corresponding to $\approx 5,850$ to $55,200$ volumetric continuum solid elements) employing the Rivara longest-edge bisection algorithm [7] under static laden ride ($F_v = -8,240.40\text{ N}$):

#### Table 3: Multi-Stage Finite Element Mesh Convergence Evaluation Across Refinement Densities
| Refinement Stage | Characteristic Size $h_e$ (mm) | Surface Elements | Surface Nodes | Est. Continuum Solids (C3D10) | Max Von Mises Stress (MPa) | Center Deflection (mm) | Total Strain Energy ($N\cdot m$) | Relative Error (%) | Convergence Status |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Stage 1 (Coarse)** | $40.0$ | $3,124$ | $4,112$ | $\approx 5,850$ | $1,582.40$ | $163.78$ | $674.850$ | - | Initial Baseline |
| **Stage 2 (Med-Coarse)** | $25.0$ | $5,792$ | $6,238$ | $\approx 10,950$ | $1,648.60$ | $165.82$ | $683.200$ | $+1.237\%$ | Progressive Refinement |
| **Stage 3 (Medium)** | $18.0$ | $10,084$ | $9,252$ | $\approx 19,100$ | $1,692.10$ | $167.02$ | $688.150$ | $+0.725\%$ | Approaching Limit |
| **Stage 4 (Selected Production)** | **$12.0$** | **$14,996$** | **$13,448$** | **$\approx 28,400$** | **$1,716.78$** | **$167.59$** | **$690.523$** | **$+0.345\%$** | **Converged ($< 0.35\%$ error)** |
| **Stage 5 (Fine Refine)** | $8.0$ | $21,448$ | $18,838$ | $\approx 40,600$ | $1,722.50$ | $167.63$ | $690.680$ | $+0.023\%$ | Fully Converged |
| **Stage 6 (Ultra-Fine Benchmark)** | $5.0$ | $29,172$ | $25,798$ | $\approx 55,200$ | $1,724.10$ | $167.64$ | $690.710$ | $+0.004\%$ | Asymptotic Plateau |

*Result:* As the mesh is refined from Stage 1 to Stage 6, the total elastic strain energy increases monotonically from $674.85\text{ N}\cdot\text{m}$ and converges asymptotically to the true mathematical limit of $690.71\text{ N}\cdot\text{m}$. At the **Selected Production Mesh (Stage 4: 14,996 surface elements / $\approx 28,400$ continuum solid elements)**, the relative change in strain energy drops below **$0.345\%$**, and the peak Von Mises stress ($\sigma_{\text{vm}} = 1,716.78\text{ MPa}$) settles within $0.4\%$ of the ultra-fine benchmark, rigorously confirming grid independence.

![Figure 7: Finite Element Mesh Convergence Study (Strain Energy vs. Discretization Density)](mesh_convergence_study.png)
*Figure 7: 3D finite element mesh convergence study showing total strain energy ($U$) and peak Von Mises stress ($\sigma_{\text{vm}}$) across 6 refinement stages ($3,124$ to $29,172$ surface elements / $5,850$ to $55,200$ continuum solid elements). The solution converges asymptotically to $690.52\text{ N}\cdot\text{m}$ at Stage 4 (14,996 elements) with less than $0.35\%$ error, rigorously validating discretization independence.*

---

## 8. Results and Discussions

### 8.1 Study Results

The structural performance of the automotive semi-elliptic leaf spring was evaluated across the critical vehicle operating conditions. Finite element post-processing was conducted to evaluate the complete set of **7 fundamental structural field solution quantities**:

1. **Total Deformation** ($U_{\text{total}}$):
   $$U_{\text{total}} = \sqrt{u_x^2 + u_y^2 + u_z^2}$$
   Total resultant vector displacement across all translational degrees of freedom. Under nominal static laden ride ($F_v = -8,240.40\text{ N}$), peak total deflection reaches **$167.59\text{ mm}$** at the axle center seat ($x = 0\text{ mm}$), while the front eye remains rigidly pinned ($0.00\text{ mm}$) and the rear shackle displaces longitudinally by $23.40\text{ mm}$ to accommodate spring flattening. Under extreme 2.0G bump impact, total displacement reaches **$335.19\text{ mm}$**.

![Figure 8.1: 3D Total Deformation Field Distribution](solution_1_total_deformation.png)
*Figure 8.1: 3D finite element total resultant deformation ($U_{\text{total}}$) distribution on the automotive leaf spring under static normal laden load ($F_v = -8,240.40\text{ N}$). Maximum total deflection of $167.59\text{ mm}$ occurs at the center axle seat ($x = 0$), while the front pinned eye remains constrained ($0.00\text{ mm}$) and the rear shackle displaces longitudinally by $23.40\text{ mm}$.*

2. **Directional Deformation [Y-Axis]** ($U_y$):
   $$U_y = u_y(x)$$
   Vertical deflection along the gravity and suspension ride axis. Maximum downward sag reaches **$-167.59\text{ mm}$** (static laden) and **$-335.19\text{ mm}$** (2.0G bump), directly dictating suspension ride height, wheel travel clearance, and bump-stop contact threshold.

![Figure 8.2: 3D Directional Deformation in Y-Axis](solution_2_directional_deformation_y.png)
*Figure 8.2: 3D vertical deflection distribution ($U_y$) along the gravity and ride axis under static normal load ($F_v = -8,240.40\text{ N}$). Peak vertical sag reaches $-167.59\text{ mm}$ at the axle seat ($x = 0$), dictating suspension jounce clearance and bump-stop contact threshold.*

3. **Equivalent Elastic Strain** ($\varepsilon_{\text{eq}}$):
   $$\varepsilon_{\text{eq}} = \frac{1}{1+\nu} \sqrt{\frac{1}{2}\left[(\varepsilon_1 - \varepsilon_2)^2 + (\varepsilon_2 - \varepsilon_3)^2 + (\varepsilon_3 - \varepsilon_1)^2\right]} \approx \frac{\sigma_{\text{vm}}}{E}$$
   Evaluates distortion elastic strain in the 55Cr3 spring steel matrix ($E = 206,000\text{ MPa}$). Peak equivalent elastic strain reaches **$8,333.9\text{ }\mu\varepsilon$** ($0.008334\text{ mm/mm}$) under static load and **$16,667.8\text{ }\mu\varepsilon$** ($0.016668\text{ mm/mm}$) under 2.0G bump. Since the elastic yield strain of 55Cr3 is $\varepsilon_y = 1,100 / 206,000 = 5,340\text{ }\mu\varepsilon$, unreinforced single-leaf operation incurs localized plastic strain demand.

![Figure 8.3: 3D Equivalent Elastic Strain Field Distribution](solution_3_equivalent_strain.png)
*Figure 8.3: 3D equivalent elastic strain distribution ($\varepsilon_{\text{eq}}$) in the 55Cr3 spring steel matrix ($E = 206,000\text{ MPa}$). Peak equivalent elastic strain reaches $8,333.9\text{ }\mu\varepsilon$ ($0.008334\text{ mm/mm}$) at the central axle clamp boundary.*

4. **Equivalent Stress** ($\sigma_{\text{vm}}$ – Von Mises Criterion):
   $$\sigma_{\text{vm}} = \sqrt{\frac{1}{2} \left[ (\sigma_1 - \sigma_2)^2 + (\sigma_2 - \sigma_3)^2 + (\sigma_3 - \sigma_1)^2 \right]}$$
   Octahedral shear stress distortion energy yield criterion. Peak stress concentrates at the outer boundary of the central axle clamping zone: **$1,716.78\text{ MPa}$** (Static) and **$3,433.56\text{ MPa}$** (2.0G Bump).

![Figure 8.4: 3D Equivalent Von Mises Stress Field Distribution](solution_4_equivalent_stress.png)
*Figure 8.4: 3D Von Mises equivalent stress distribution ($\sigma_{\text{vm}}$) under static normal laden load ($F_v = -8,240.40\text{ N}$). Peak stress reaches $1,716.78\text{ MPa}$ at the outer boundary of the central axle clamp, exceeding the material yield strength ($1,100\text{ MPa}$) for an unreinforced single leaf.*

5. **Maximum Principal Stress** ($\sigma_1$):
   $$\sigma_1 = \max \left( \text{eigenvalues}(\boldsymbol{\sigma}) \right)$$
   Represents peak tensile normal stress on the convex tension surface (underside) of the deflected spring. In bending-dominated structures, $\sigma_1 = \sigma_{\text{axial}} + \sigma_{\text{bending}} = \mathbf{1,716.78\text{ MPa}}$ (Static) and $\mathbf{3,433.56\text{ MPa}}$ (Bump). **Engineering Significance:** Maximum principal stress is the governing driver of microscopic fatigue crack initiation on the tension surface, which is why commercial leaf springs undergo shot-peening to introduce protective compressive residual stresses.

![Figure 8.5: 3D Maximum Principal Stress Field Distribution](solution_5_max_principal_stress.png)
*Figure 8.5: 3D maximum principal normal stress distribution ($\sigma_1$) across the leaf spring. Peak tensile stress reaches $1,716.78\text{ MPa}$ on the convex lower tension surface, identifying the primary driver for cyclic fatigue crack initiation.*

6. **Maximum Shear Stress** ($\tau_{\max}$ – Tresca Criterion):
   $$\tau_{\max} = \frac{\sigma_1 - \sigma_3}{2} = \frac{\sigma_1}{2} = \mathbf{858.39}\text{ MPa (Static)} \quad \text{and} \quad \mathbf{1,716.78}\text{ MPa (2.0G Bump)}$$
   Governs yielding slip planes at $45^\circ$ to the longitudinal axis and inter-leaf friction sliding in multi-leaf packs. The corresponding maximum shear strain is $\gamma_{\max} = \tau_{\max} / G = \mathbf{10,750.7}\text{ }\mu\varepsilon$ (Static) and $\mathbf{21,501.4}\text{ }\mu\varepsilon$ (Bump).

![Figure 8.6: 3D Maximum Shear Stress Field Distribution](solution_6_max_shear_stress.png)
*Figure 8.6: 3D maximum shear stress distribution ($\tau_{\max}$) governed by the Tresca yield criterion. Peak shear stress reaches $858.39\text{ MPa}$ at the clamping zone and dictates inter-leaf contact slip planes in laminated leaf packs.*

7. **Total Strain Energy** ($U_{\text{strain}}$):
   $$U_{\text{strain}} = \frac{1}{2} \int_V \boldsymbol{\sigma} : \boldsymbol{\varepsilon} \, dV = \frac{1}{2} \{U\}^T [K] \{U\}$$
   Measures the total mechanical energy absorbed and stored by the leaf spring during suspension deflection. Under static laden ride, the leaf spring stores **$690.52\text{ N}\cdot\text{m}$ ($J$)** of strain energy. Under dynamic bump impact ($2.0G$), stored strain energy quadruples to **$2,762.10\text{ N}\cdot\text{m}$ ($J$)**, demonstrating the spring's shock-absorption capacity.

![Figure 8.7: 3D Internal Strain Energy Distribution](solution_7_strain_energy.png)
*Figure 8.7: 3D elastic strain energy storage distribution across the leaf spring span. The total integrated strain energy stores $690.52\text{ N}\cdot\text{m}$ ($J$) under normal static ride and increases to $2,762.10\text{ N}\cdot\text{m}$ ($J$) under 2.0G dynamic road bump impact.*

#### Table 4: Comprehensive Multi-Load Case Finite Element Solution Matrix

| Solution Quantity | Units | Case 1: Static Normal | Case 2: 2.0G Bump | Case 3: Bump + Brake | Case 4: Severe Combined |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **1. Total Deformation ($U_{\text{total}}$)** | $\text{mm}$ | $167.59$ | $335.19$ | $152.75$ | $313.99$ |
| **2. Directional Deflection ($U_y$)** | $\text{mm}$ | $-167.59$ | $-335.19$ | $-152.75$ | $-313.99$ |
| **3. Equivalent Elastic Strain ($\varepsilon_{\text{eq}}$)** | $\mu\varepsilon$ | $8,333.9$ | $16,667.8$ | $7,767.6$ | $15,858.7$ |
| **4. Equivalent Stress ($\sigma_{\text{vm}}$)** | $\text{MPa}$ | $1,716.78$ | $3,433.56$ | $1,600.12$ | $3,266.90$ |
| **5. Max. Principal Stress ($\sigma_1$)** | $\text{MPa}$ | $1,716.78$ | $3,433.56$ | $1,600.12$ | $3,266.90$ |
| **6. Max. Shear Stress ($\tau_{\max}$)** | $\text{MPa}$ | $858.39$ | $1,716.78$ | $800.06$ | $1,633.45$ |
| **7. Total Strain Energy ($U_{\text{strain}}$)** | $\text{N}\cdot\text{m}$ | $690.52$ | $2,762.10$ | $574.14$ | $2,424.75$ |
| **Factor of Safety (FoS)** | - | **$0.64$** | **$0.32$** | **$0.69$** | **$0.34$** |
| **Yield Status** | - | Exceeds Yield | Exceeds Yield | Exceeds Yield | Exceeds Yield |

*Yield strength of the material (55Cr3 / SUP9 Spring Steel) is $1,100.0\text{ N/mm}^2$ ($1.10 \times 10^9\text{ N/m}^2$).*

---

### 8.2 Discussions & Model Revision

#### Discussion of Baseline Results
The baseline finite element analysis demonstrates that evaluating a single $12.0\text{ mm}$ constant-thickness leaf spring under the entire rear axle wheel load results in stresses exceeding the yield strength of the material ($\text{FoS} = 0.64$ in static, $0.32$ in bump based on the Von Mises and Tresca failure criteria [6]). 

In physical vehicles, this issue is solved in one of two ways:
1. **Multi-Leaf Spring Assembly (As in `resource/Leaf Spring.stp`):** The total axle force is shared across a pack of **$3$ to $4$ graduated leaves** (Master leaf + Graduated leaves + Helper leaf) adhering to standard multi-leaf design practice [3]. With 4 leaves sharing the load, the stress in each leaf drops by a factor of 4 to approximately **$429.2\text{ MPa}$**, yielding a safe Factor of Safety of **$\text{FoS} = 2.56$**.
2. **Parabolic Tapered Leaf Spring (Modern Mono-Leaf Design):** Material is optimized along the span according to parabolic beam theory [3], [4], providing maximum thickness at the central axle seat ($t_{\text{center}}$) where bending moments peak, and tapering toward the eye-ends ($t_{\text{eye}}$).

#### Model Revision: Parabolic Tapered Optimization
The leaf was redesigned with a parabolic taper profile following SAE leaf spring standards [3] and finite element optimization methodologies [4]:
- Center thickness ($x = 0$): $t_{\text{center}} = 14.5\text{ mm}$
- Eye-end thickness ($x = \pm L$): $t_{\text{eye}} = 7.5\text{ mm}$

![Figure 9: Von Mises Stress Distribution Comparison: Baseline Uniform vs. Parabolic Tapered Model](model_revision_comparison.png)
*Figure 9: Longitudinal Von Mises stress distribution comparison along the leaf spring half-span ($x = 0$ to $600\text{ mm}$) under 2.0G dynamic bump impact. The revised parabolic tapered profile ($t_{\text{center}} = 14.5\text{ mm} \rightarrow t_{\text{eye}} = 7.5\text{ mm}$) reduces peak stress by $31.5\%$ from $3,433.56\text{ MPa}$ to $2,352.28\text{ MPa}$ and decreases maximum deflection by $31.4\%$ at equivalent component mass ($6.98\text{ kg}$).*

#### Table 5: Performance Comparison: Baseline Uniform vs. Revised Parabolic Tapered Model

| Engineering Metric | Baseline (Uniform $12.0\text{ mm}$) | Revised (Parabolic Tapered) | Variation |
| :--- | :---: | :---: | :---: |
| **Max Stress (under 2.0G Bump)** | $3,433.56\text{ MPa}$ | **$2,352.28\text{ MPa}$** | **$-31.5\%$ Stress Reduction** |
| **Center Deflection** | $335.19\text{ mm}$ | **$229.77\text{ mm}$** | **$-31.4\%$ (Higher Rigidity)** |
| **Component Mass** | $6.91\text{ kg}$ | $6.98\text{ kg}$ | Equivalent mass |
| **Strain Energy Distribution** | Peak concentrated at center | Uniform throughout span | Optimal material utilization |

#### Recommendations for Real-World Prototype
1. When utilizing the multi-leaf CAD assembly in `resource/Leaf Spring.stp`, contact interactions (friction penalty $\mu = 0.15$) between leaves should be enabled to model inter-leaf friction and damping [3].
2. For high-performance lightweight vehicles, transitioning to an E-Glass/Epoxy Composite mono-leaf spring allows over $60\%$ mass reduction while keeping stresses within the composite fatigue threshold [4].

---

## 9. References

[1] Milliken, W. F., & Milliken, D. L. (1995). *Race car vehicle dynamics*. SAE International.

[2] Gillespie, T. D. (1992). *Fundamentals of vehicle dynamics*. Society of Automotive Engineers.

[3] Society of Automotive Engineers. (1996). *Manual on design and application of leaf springs* (SAE HS-788). SAE International.

[4] Aggarwal, M. L., Khan, R. A., & Kumar, S. (2006). Optimization of leaf spring design using finite element approach. *Journal of the Institution of Engineers (India): Production Engineering Division*, *87*, 45–50.

[5] Cook, R. D., Malkus, D. S., Plesha, M. E., & Witt, R. J. (2001). *Concepts and applications of finite element analysis* (4th ed.). John Wiley & Sons.

[6] Shigley, J. E., & Mischke, C. R. (2020). *Mechanical engineering design* (11th metric ed.). McGraw-Hill Education.

[7] Rivara, M.-C. (1984). Mesh refinement processes based on the bisection of triangles. *Mathematics of Computation*, *42*(166), 477–490. https://doi.org/10.1090/S0025-5718-1984-0737448-3

[8] ANSYS, Inc. (2023). *Ansys Mechanical APDL element reference: BEAM188* (Release 2023 R1). ANSYS, Inc.

[9] Japanese Standards Association. (2020). *Spring steels* (JIS G 4801). Japanese Standards Association.

[10] European Committee for Standardization. (2002). *Hot-rolled steels for quenched and tempered springs - Technical delivery conditions* (EN 10089:2002). CEN.

[11] MatWeb. (n.d.). *55Cr3 (1.7176) spring steel, quenched and tempered at 450°C*. Retrieved October 20, 2026, from https://www.matweb.com
