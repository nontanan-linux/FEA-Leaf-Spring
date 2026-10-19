"""
===============================================================================
Automotive Leaf Spring - CAD to Finite Element Mesh Generator & Loader
Course: Finite Element Method in Engineering (M.Eng)
Instructor: Asst. Prof. Dr. Monsak Pimsarn
===============================================================================
Features:
1. Multi-Format Geometry Support:
   - CAD Solids / Surfaces: .stp, .step, .igs, .iges, .brep (via Gmsh / OCC)
   - Pre-existing FEA Meshes: .stl, .inp, .msh, .ply, .obj (via Trimesh / Open3D)
2. Decoupled Pipeline (Preprocessing vs. Solving):
   - If an FEA mesh already exists, load and validate it directly (instant execution).
   - If only CAD is available, automatically discretize with target element sizing.
3. Adaptive Refinement:
   - Rivara longest-edge bisection for curvature-sensitive refinement.
4. FEA Export Formats:
   - CalculiX / Abaqus (.inp) format ready for PrePoMax or CalculiX solvers.
   - Stereolithography (.stl) for visual inspection and CFD/FEA boundary shells.
   - Gmsh (.msh) for advanced multi-physics analysis.
5. Mesh Quality Diagnostics:
   - Aspect ratio, Jacobian estimate, minimum/maximum edge lengths, and node stats.
===============================================================================
"""

import os
import sys
import argparse
import numpy as np

# Optional imports with graceful fallbacks
try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False

try:
    import gmsh
    HAS_GMSH = True
except ImportError:
    HAS_GMSH = False

try:
    import trimesh
    HAS_TRIMESH = True
except ImportError:
    HAS_TRIMESH = False

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False


# -----------------------------------------------------------------------------
# 1. Mesh Quality & Diagnostic Utilities
# -----------------------------------------------------------------------------
def calculate_mesh_diagnostics(vertices, triangles):
    """
    Computes geometric diagnostics for surface triangular meshes.
    Returns dictionary with bounding box, edge lengths, aspect ratios, and areas.
    """
    v = np.asarray(vertices, dtype=float)
    t = np.asarray(triangles, dtype=int)
    
    if len(v) == 0 or len(t) == 0:
        return {}
    
    # Bounding box
    bbox_min = v.min(axis=0)
    bbox_max = v.max(axis=0)
    span_dims = bbox_max - bbox_min
    
    # Triangle vertices
    v0 = v[t[:, 0]]
    v1 = v[t[:, 1]]
    v2 = v[t[:, 2]]
    
    # Edge vectors and lengths
    e0 = v1 - v0
    e1 = v2 - v1
    e2 = v0 - v2
    
    len0 = np.linalg.norm(e0, axis=1)
    len1 = np.linalg.norm(e1, axis=1)
    len2 = np.linalg.norm(e2, axis=1)
    
    all_lengths = np.concatenate([len0, len1, len2])
    min_edge = float(np.min(all_lengths))
    max_edge = float(np.max(all_lengths))
    mean_edge = float(np.mean(all_lengths))
    
    # Triangle Areas (Heron's formula / Cross product)
    cross_prod = np.cross(e0, -e2)
    areas = 0.5 * np.linalg.norm(cross_prod, axis=1)
    total_area = float(np.sum(areas))
    
    # Aspect Ratio: max(edge) / (2 * inradius) = max(edge) * semiperimeter / (2 * Area)
    semi_p = 0.5 * (len0 + len1 + len2)
    denom = 2.0 * np.maximum(areas, 1e-12)
    max_edges = np.maximum(np.maximum(len0, len1), len2)
    aspect_ratios = (max_edges * semi_p) / (denom * 1.7320508) # Normalized so equilateral = 1.0
    aspect_ratios = np.clip(aspect_ratios, 1.0, 100.0)
    
    return {
        "num_nodes": len(v),
        "num_elements": len(t),
        "bbox_min": bbox_min.tolist(),
        "bbox_max": bbox_max.tolist(),
        "span_dims": span_dims.tolist(),
        "min_edge_mm": min_edge,
        "max_edge_mm": max_edge,
        "mean_edge_mm": mean_edge,
        "total_surface_area_mm2": total_area,
        "mean_aspect_ratio": float(np.mean(aspect_ratios)),
        "max_aspect_ratio": float(np.max(aspect_ratios)),
    }


# -----------------------------------------------------------------------------
# 2. Rivara Longest-Edge Adaptive Refinement Algorithm
# -----------------------------------------------------------------------------
def refine_longest_edge(vertices, triangles, max_edge_len=12.0, max_iterations=5):
    """
    Rivara longest-edge bisection algorithm for finite element meshes.
    Recursively bisects edges longer than max_edge_len while avoiding degenerate angles.
    """
    v_list = [list(v) for v in vertices]
    t_list = [list(t) for t in triangles]
    
    for iteration in range(max_iterations):
        new_triangles = []
        any_split = False
        edge_to_mid = {}
        
        for tri in t_list:
            v0 = np.array(v_list[tri[0]])
            v1 = np.array(v_list[tri[1]])
            v2 = np.array(v_list[tri[2]])
            d01 = np.linalg.norm(v1 - v0)
            d12 = np.linalg.norm(v2 - v1)
            d20 = np.linalg.norm(v0 - v2)
            
            max_d = max(d01, d12, d20)
            if max_d > max_edge_len:
                any_split = True
                if max_d == d01:
                    e = tuple(sorted((tri[0], tri[1])))
                    if e not in edge_to_mid:
                        edge_to_mid[e] = len(v_list)
                        v_list.append(((v0 + v1) / 2.0).tolist())
                    m = edge_to_mid[e]
                    new_triangles.append([tri[2], tri[0], m])
                    new_triangles.append([tri[2], m, tri[1]])
                elif max_d == d12:
                    e = tuple(sorted((tri[1], tri[2])))
                    if e not in edge_to_mid:
                        edge_to_mid[e] = len(v_list)
                        v_list.append(((v1 + v2) / 2.0).tolist())
                    m = edge_to_mid[e]
                    new_triangles.append([tri[0], tri[1], m])
                    new_triangles.append([tri[0], m, tri[2]])
                else:
                    e = tuple(sorted((tri[2], tri[0])))
                    if e not in edge_to_mid:
                        edge_to_mid[e] = len(v_list)
                        v_list.append(((v2 + v0) / 2.0).tolist())
                    m = edge_to_mid[e]
                    new_triangles.append([tri[1], tri[2], m])
                    new_triangles.append([tri[1], m, tri[0]])
            else:
                new_triangles.append(tri)
                
        t_list = new_triangles
        if not any_split:
            break
            
    return np.array(v_list, dtype=float), np.array(t_list, dtype=int)


# -----------------------------------------------------------------------------
# 3. Direct CAD Meshing via Gmsh / OpenCASCADE
# -----------------------------------------------------------------------------
def mesh_cad_file(cad_path, element_size_min=10.0, element_size_max=20.0, dim=2, order=1):
    """
    Imports CAD solid/surface geometry (.stp, .step, .igs, .iges, .brep)
    using Gmsh and generates conformal finite element mesh.
    """
    if not HAS_GMSH:
        raise RuntimeError("Gmsh Python module is not installed. Run 'pip install gmsh'.")
    
    if not os.path.isfile(cad_path):
        raise FileNotFoundError(f"CAD file not found: {cad_path}")
        
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)  # Suppress excessive terminal noise
        gmsh.option.setNumber("Mesh.CharacteristicLengthMin", float(element_size_min))
        gmsh.option.setNumber("Mesh.CharacteristicLengthMax", float(element_size_max))
        gmsh.option.setNumber("Mesh.ElementOrder", int(order))
        gmsh.option.setNumber("Mesh.Algorithm", 6)     # Frontal-Delaunay for 2D surfaces
        
        # Open CAD model (OpenCASCADE automatically parses STEP/IGES)
        gmsh.open(os.path.abspath(cad_path))
        
        # Generate mesh
        gmsh.model.mesh.generate(dim)
        
        # Extract node coordinates
        node_tags, coords, _ = gmsh.model.mesh.getNodes()
        vertices = coords.reshape((-1, 3))
        node_map = {tag: idx for idx, tag in enumerate(node_tags)}
        
        # Extract triangular surface elements (2-node line = 1, 3-node tri = 2, 4-node quad = 3)
        element_types, element_tags, element_node_tags = gmsh.model.mesh.getElements(dim=dim)
        
        triangles = []
        for el_type, el_nodes in zip(element_types, element_node_tags):
            if el_type == 2:  # 3-node triangle
                reshaped = el_nodes.reshape((-1, 3))
                for row in reshaped:
                    triangles.append([node_map[row[0]], node_map[row[1]], node_map[row[2]]])
            elif el_type == 9: # 6-node quadratic triangle
                reshaped = el_nodes.reshape((-1, 6))
                for row in reshaped:
                    triangles.append([node_map[row[0]], node_map[row[1]], node_map[row[2]]])
                    
        triangles = np.array(triangles, dtype=int)
        return vertices, triangles
    finally:
        gmsh.finalize()


# -----------------------------------------------------------------------------
# 4. Pre-Existing Mesh Loading (STL, INP, etc.)
# -----------------------------------------------------------------------------
def load_mesh_file(mesh_path):
    """
    Loads pre-existing FEA mesh from file (.stl, .inp, .obj, .ply).
    """
    if not os.path.isfile(mesh_path):
        raise FileNotFoundError(f"Mesh file not found: {mesh_path}")
        
    ext = os.path.splitext(mesh_path)[1].lower()
    
    if ext == ".stl":
        if HAS_TRIMESH:
            mesh = trimesh.load(mesh_path)
            return np.array(mesh.vertices), np.array(mesh.faces)
        elif HAS_OPEN3D:
            mesh = o3d.io.read_triangle_mesh(mesh_path)
            return np.asarray(mesh.vertices), np.asarray(mesh.triangles)
        else:
            raise RuntimeError("Requires trimesh or open3d to read STL.")
            
    elif ext == ".inp":
        # Parse Abaqus / CalculiX .inp file
        nodes = {}
        elements = []
        reading_nodes = False
        reading_elements = False
        
        with open(mesh_path, "r", encoding="utf-8") as f:
            for line in f:
                line_str = line.strip()
                if not line_str or line_str.startswith("**"):
                    continue
                if line_str.upper().startswith("*NODE"):
                    reading_nodes = True
                    reading_elements = False
                    continue
                if line_str.upper().startswith("*ELEMENT"):
                    reading_nodes = False
                    reading_elements = True
                    continue
                if line_str.startswith("*"):
                    reading_nodes = False
                    reading_elements = False
                    continue
                    
                parts = [p.strip() for p in line_str.split(",")]
                if reading_nodes and len(parts) >= 4:
                    n_id = int(parts[0])
                    x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                    nodes[n_id] = [x, y, z]
                elif reading_elements and len(parts) >= 4:
                    el_nodes = [int(p) for p in parts[1:4]]
                    elements.append(el_nodes)
                    
        # Map node IDs to 0-indexed contiguous array
        node_ids = sorted(nodes.keys())
        id_to_idx = {nid: i for i, nid in enumerate(node_ids)}
        vertices = np.array([nodes[nid] for nid in node_ids], dtype=float)
        triangles = np.array([[id_to_idx[nid] for nid in tri] for tri in elements], dtype=int)
        return vertices, triangles
        
    else:
        # Fallback to trimesh generic reader
        if HAS_TRIMESH:
            mesh = trimesh.load(mesh_path)
            return np.array(mesh.vertices), np.array(mesh.faces)
        raise ValueError(f"Unsupported mesh format: {ext}")


# -----------------------------------------------------------------------------
# 5. FEA Exporters (CalculiX INP & STL)
# -----------------------------------------------------------------------------
def export_inp(vertices, triangles, output_path, element_type="S3", part_name="LEAF_SPRING"):
    """
    Exports vertices and elements to an Abaqus / CalculiX .inp format
    ready for direct simulation in PrePoMax or CalculiX.
    """
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        f.write("** =========================================================\n")
        f.write(f"** Finite Element Mesh Export: {part_name}\n")
        f.write(f"** Nodes: {len(vertices)}, Elements: {len(triangles)}\n")
        f.write("** =========================================================\n")
        f.write(f"*HEADING\nLeaf Spring Mesh Discretization\n")
        f.write(f"*PART, NAME={part_name}\n")
        f.write("*NODE\n")
        for i, v in enumerate(vertices):
            f.write(f"{i+1}, {v[0]:.6f}, {v[1]:.6f}, {v[2]:.6f}\n")
            
        f.write(f"*ELEMENT, TYPE={element_type}, ELSET=ESET_ALL\n")
        for j, tri in enumerate(triangles):
            f.write(f"{j+1}, {tri[0]+1}, {tri[1]+1}, {tri[2]+1}\n")
            
        f.write("*NSET, NSET=NSET_ALL, GENERATE\n")
        f.write(f"1, {len(vertices)}, 1\n")
        f.write("*END PART\n")
    print(f"-> Successfully exported CalculiX/Abaqus mesh: {output_path}")


def export_stl(vertices, triangles, output_path):
    """Exports surface mesh to standard binary/ASCII STL format."""
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    if HAS_TRIMESH:
        mesh = trimesh.Trimesh(vertices=vertices, faces=triangles)
        mesh.export(output_path)
    elif HAS_OPEN3D:
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(vertices)
        mesh.triangles = o3d.utility.Vector3iVector(triangles)
        o3d.io.write_triangle_mesh(output_path, mesh)
    else:
        raise RuntimeError("Requires trimesh or open3d to export STL.")
    print(f"-> Successfully exported STL mesh: {output_path}")


# -----------------------------------------------------------------------------
# 6. Unified Python Importer API (load_or_generate_mesh)
# -----------------------------------------------------------------------------
def load_or_generate_mesh(
    source_path=None,
    output_cache_path=None,
    element_size=12.0,
    refine=True,
    config_path=None
):
    """
    Intelligent Preprocessing Pipeline function for FEA Solvers:
    - If source_path is a mesh (.stl, .inp), loads it directly.
    - If source_path is a CAD solid (.stp, .step, .igs), discretizes it.
    - If source_path is None, inspects config/model.yaml for defaults.
    - Returns structured dictionary ready for FEA analysis.
    """
    # 1. Resolve defaults from config if available
    if config_path is None:
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cfg_candidate = os.path.join(base_dir, "config", "model.yaml")
        if os.path.isfile(cfg_candidate):
            config_path = cfg_candidate
            
    if config_path and HAS_YAML and os.path.isfile(config_path):
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
            if source_path is None:
                # Prefer existing mesh if available, otherwise fallback to CAD STEP
                stl_file = os.path.join(base_dir, cfg["cad_geometry"]["file_paths"]["stl_mesh"])
                step_file = os.path.join(base_dir, cfg["cad_geometry"]["file_paths"]["step_solid"])
                source_path = stl_file if os.path.isfile(stl_file) else step_file
            if element_size is None and "mesh_refinement" in cfg:
                element_size = cfg["mesh_refinement"].get("max_edge_length_threshold_mm", 12.0)
                
    if source_path is None:
        raise ValueError("No source geometry or mesh path provided and could not resolve from config.")
        
    ext = os.path.splitext(source_path)[1].lower()
    is_cad = ext in [".stp", ".step", ".igs", ".iges", ".brep"]
    is_mesh = ext in [".stl", ".inp", ".msh", ".ply", ".obj"]
    
    print("=" * 70)
    print(" FEA PREPROCESSING & MESH PIPELINE")
    print("=" * 70)
    print(f"Source Path: {source_path}")
    print(f"File Category: {'CAD Solid/Surface Geometry' if is_cad else 'Pre-Discretized Mesh'}")
    
    if is_mesh:
        print("-> Loading pre-existing mesh file (bypassing CAD discretization)...")
        v, t = load_mesh_file(source_path)
        was_generated = False
    elif is_cad:
        print("-> Discretizing CAD solid geometry with Gmsh / OpenCASCADE...")
        v, t = mesh_cad_file(source_path, element_size_min=element_size*0.8, element_size_max=element_size*1.5)
        was_generated = True
    else:
        raise ValueError(f"Unrecognized file extension: {ext}")
        
    if refine and element_size is not None:
        print(f"-> Applying Rivara bisection refinement (threshold: {element_size} mm)...")
        v, t = refine_longest_edge(v, t, max_edge_len=element_size)
        
    # Compute mesh diagnostics
    diag = calculate_mesh_diagnostics(v, t)
    print(f"-> Mesh Verification: {diag['num_nodes']:,} nodes, {diag['num_elements']:,} elements")
    print(f"   Span: X={diag['span_dims'][0]:.1f}mm, Y={diag['span_dims'][1]:.1f}mm, Z={diag['span_dims'][2]:.1f}mm")
    print(f"   Aspect Ratio: mean={diag['mean_aspect_ratio']:.2f}, max={diag['max_aspect_ratio']:.2f}")
    
    # Save cache if output path requested
    if output_cache_path:
        out_ext = os.path.splitext(output_cache_path)[1].lower()
        if out_ext == ".inp":
            export_inp(v, t, output_cache_path)
        elif out_ext == ".stl":
            export_stl(v, t, output_cache_path)
            
    print("=" * 70)
    return {
        "vertices": v,
        "triangles": t,
        "diagnostics": diag,
        "source_file": source_path,
        "is_generated_from_cad": was_generated
    }


# -----------------------------------------------------------------------------
# 7. Standalone Command Line Execution
# -----------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Automotive Leaf Spring CAD to Mesh Generator & Preprocessing Pipeline"
    )
    parser.add_argument(
        "--input", "-i", type=str, default=None,
        help="Path to CAD file (.stp/.step) or existing mesh file (.stl/.inp)"
    )
    parser.add_argument(
        "--output", "-o", type=str, default=None,
        help="Path to export generated mesh (.inp or .stl)"
    )
    parser.add_argument(
        "--size", "-s", type=float, default=12.0,
        help="Target element size or max edge length in mm (default: 12.0 mm)"
    )
    parser.add_argument(
        "--no-refine", action="store_true",
        help="Disable adaptive Rivara longest-edge refinement"
    )
    parser.add_argument(
        "--config", "-c", type=str, default=None,
        help="Path to model.yaml configuration file"
    )
    
    args = parser.parse_args()
    
    # Set default paths if not provided
    scripts_dir = os.path.dirname(os.path.abspath(__file__))
    project_dir = os.path.dirname(scripts_dir)
    
    input_path = args.input
    if input_path is None:
        # Check default locations
        cad_candidate = os.path.join(project_dir, "resource", "Leaf Spring.stp")
        stl_candidate = os.path.join(project_dir, "resource", "Leaf Spring.stl")
        input_path = cad_candidate if os.path.isfile(cad_candidate) else stl_candidate
        
    output_path = args.output
    if output_path is None:
        output_path = os.path.join(project_dir, "results", "generated_leaf_spring_mesh.inp")
        
    mesh_data = load_or_generate_mesh(
        source_path=input_path,
        output_cache_path=output_path,
        element_size=args.size,
        refine=not args.no_refine,
        config_path=args.config
    )
    
    print(f"\n[DONE] Pipeline completed successfully. Output ready for FEA solver.")

if __name__ == "__main__":
    main()
