from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from planning.perception.object_pose_registration import ObjectPoseRegistrar

import open3d as o3d


_DEFAULT_MESH = (
    Path(__file__).resolve().parent.parent
    / "perception"
    / "assets"
    / "meshes"
    / "Cup.stl"
)

_DEFAULT_CLOUD = (
    Path(__file__).resolve().parent.parent
    / "perception"
    / "assets"
    / "point_clouds"
    / "cup.xyz"
)

_DEFAULT_CONFIG = (
    Path(__file__).resolve().parent.parent
    / "config"
    / "registration_icp.yaml"
)


def _parse_args() -> argparse.Namespace:
    """
    Parse CLI arguments for the ICP cup pose registration demo.

    Returns:
        (argparse.Namespace): Parsed command-line arguments including mesh path,
            point cloud path, config path, and visualization flag.
    """
    parser = argparse.ArgumentParser(description="ICP-based pose registration demo for the cup.")
    parser.add_argument("--mesh", type=str, default=str(_DEFAULT_MESH), help="Path to mesh file.")
    parser.add_argument("--point_cloud", type=str, default=str(_DEFAULT_CLOUD), help="Path to .xyz point cloud.")
    parser.add_argument("--config", type=str, default=str(_DEFAULT_CONFIG), help="Path to registration config YAML.")
    parser.add_argument("--visualize", action="store_true", help="Visualize alignment in Open3D viewer.")
    parser.add_argument("--debug_preprocess", action="store_true", help="Visualize point cloud after filtering/downsampling.")
    parser.add_argument("--debug_init", action="store_true", help="Visualize initial guess alignment before ICP.")

    return parser.parse_args()


def main() -> None:
    """
    Run ICP-based pose registration for a given mesh and observed XYZ point cloud.

    This loads the input point cloud, registers it to the mesh using ObjectPoseRegistrar,
    prints registration metrics and the estimated transform, and optionally visualizes
    the final alignment in an Open3D viewer.

    Returns:
        (None): This function prints results to stdout and may open a visualization window.
    """
    args = _parse_args()

    registrar = ObjectPoseRegistrar(config_path=args.config)
    cloud = registrar.load_xyz_point_cloud(args.point_cloud)
    if cloud.shape[0] == 0:
        raise ValueError("Loaded empty point cloud.")
    
    if args.debug_preprocess or args.debug_init:
        # Build Open3D cloud
        cloud_pcd = o3d.geometry.PointCloud()
        cloud_pcd.points = o3d.utility.Vector3dVector(cloud.astype(np.float64))

        mesh = registrar._load_mesh(args.mesh)                 # ok for demo/debug
        mesh_pcd = registrar._mesh_to_point_cloud(mesh)        # deterministic if you changed sampling

        mesh_clean, mesh_down, _ = registrar._preprocess(mesh_pcd, for_mesh=True)
        cloud_clean, cloud_down, _ = registrar._preprocess(cloud_pcd, for_mesh=False)

        if args.debug_preprocess:
            cloud_pcd.paint_uniform_color([0.8, 0.0, 0.0])     # raw
            cloud_clean.paint_uniform_color([0.0, 0.0, 0.8])   # filtered
            cloud_down.paint_uniform_color([0.6, 0.0, 0.6])    # downsampled
            o3d.visualization.draw_geometries([cloud_pcd, cloud_clean, cloud_down])

        if args.debug_init:
            init_T = registrar._estimate_pose_pca(cloud_down)

            init_candidates = registrar._generate_init_candidates(init_T)
            # show first candidate only (base) to keep it simple
            mesh_init = o3d.geometry.PointCloud(mesh_down)
            mesh_init.paint_uniform_color([0.0, 0.7, 0.0])
            cloud_down.paint_uniform_color([0.8, 0.0, 0.0])

            mesh_init.transform(np.asarray(init_candidates[0], dtype=np.float64))
            o3d.visualization.draw_geometries([mesh_init, cloud_down])


    try:
        result = registrar.register_point_cloud_to_mesh(
            object_point_cloud=cloud,
            mesh_path=args.mesh,
        )
    except ValueError as e:
        print(str(e))
        return

    np.set_printoptions(precision=6, suppress=True)
    print("=== Registration Result ===")
    print(f"Mesh:        {args.mesh}")
    print(f"Point cloud: {args.point_cloud}")
    print(f"Fitness:     {result.fitness:.4f}")
    print(f"Inlier RMSE: {result.inlier_rmse:.6f} m")
    print("T_mesh_to_cloud:")
    print(result.T_mesh_to_cloud)

    # Print translation + rotation magnitude for acceptance criteria (~2cm, ~10deg)
    R = result.T_mesh_to_cloud[:3, :3]
    t = result.T_mesh_to_cloud[:3, 3]

    trace = float(np.trace(R))
    cos_theta = (trace - 1.0) / 2.0
    cos_theta = float(np.clip(cos_theta, -1.0, 1.0))
    theta_deg = float(np.degrees(np.arccos(cos_theta)))

    print(f"Translation (m): {t}")
    print(f"Rotation magnitude (deg): {theta_deg:.2f}")

    if args.visualize:
        registrar.visualize_alignment(
            object_point_cloud=cloud,
            mesh_path=args.mesh,
            T_mesh_to_cloud=result.T_mesh_to_cloud,
        )


if __name__ == "__main__":
    main()
