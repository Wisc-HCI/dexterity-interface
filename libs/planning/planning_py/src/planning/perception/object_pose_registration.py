from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

try:
    import open3d as o3d
except Exception as exc:  # pragma: no cover
    o3d = None


_DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "registration_icp.yaml"


@dataclass(frozen=True)
class RegistrationResult:
    """
    Container for object pose registration output.

    Attributes:
        T_mesh_to_cloud (np.ndarray): (4, 4) homogeneous transform that maps points from the
            mesh frame into the observed point cloud frame.
        fitness (float): ICP/RANSAC fitness score (fraction of inliers).
        inlier_rmse (float): Inlier RMSE in meters.
    """
    T_mesh_to_cloud: np.ndarray
    fitness: float
    inlier_rmse: float


class ObjectPoseRegistrar:
    """
    Estimate object pose by registering an observed point cloud to a known object mesh.

    The typical pipeline is:
      1) Load mesh -> sample to point cloud
      2) Preprocess (downsample + normals + optional outlier removal)
      3) (Optional) Coarse alignment via RANSAC/FPFH
      4) Refine with ICP
    """

    def __init__(self, config_path: str | Path | None = None):
        """
        Args:
            config_path (str | Path | None): Path to a YAML config file controlling registration
                parameters. When None, uses the package default config.
        """
        if o3d is None:
            raise ImportError("open3d not found. Please `pip install open3d` to use ICP registration.")

        self._config_path = Path(config_path) if config_path is not None else _DEFAULT_CONFIG_PATH
        self._cfg = self._load_config(self._config_path)

    @staticmethod
    def _load_config(config_path: Path) -> dict[str, Any]:
        """
        Load YAML configuration.

        Args:
            config_path (Path): Path to YAML file.

        Returns:
            (dict[str, Any]): Parsed config dictionary.
        """
        if not config_path.exists():
            raise FileNotFoundError(f"Registration config not found: {config_path}")

        with config_path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        return data

    @staticmethod
    def load_xyz_point_cloud(xyz_path: str | Path) -> np.ndarray:
        """
        Load an XYZ point cloud file (one point per line).

        Expected format:
            x y z

        Args:
            xyz_path (str | Path): Path to .xyz file.

        Returns:
            (np.ndarray): (N, 3) point cloud XYZ in meters.
        """
        path = Path(xyz_path).expanduser()
        pts: list[list[float]] = []

        with path.open("r", encoding="utf-8") as f:
            for line in f:
                s = line.strip()
                if not s:
                    continue
                parts = s.split()
                if len(parts) < 3:
                    continue
                try:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                except ValueError:
                    continue
                if not np.isfinite([x, y, z]).all():
                    continue
                pts.append([x, y, z])

        if len(pts) == 0:
            return np.empty((0, 3), dtype=np.float32)

        return np.asarray(pts, dtype=np.float32)

    def register_point_cloud_to_mesh(
        self,
        object_point_cloud: np.ndarray,
        mesh_path: str | Path,
    ) -> RegistrationResult:
        """
        Register an observed object point cloud to a known mesh.

        Args:
            object_point_cloud (np.ndarray): (N, 3) observed object point cloud in meters.
            mesh_path (str | Path): Path to object mesh (.stl/.obj/.ply). Mesh may be in mm; the
                config `mesh_unit_scale` is applied.

        Returns:
            (RegistrationResult): Estimated pose as a (4, 4) transform plus quality metrics.
        """
        if object_point_cloud is None:
            raise ValueError("`object_point_cloud` must be provided")

        pts = np.asarray(object_point_cloud, dtype=np.float32)
        if pts.ndim != 2 or pts.shape[1] != 3:
            raise ValueError("`object_point_cloud` must have shape (N, 3)")
        if pts.shape[0] < 30:
            raise ValueError("`object_point_cloud` too small; need at least ~30 points")

        mesh = self._load_mesh(mesh_path)
        mesh_pcd = self._mesh_to_point_cloud(mesh)

        cloud_pcd = o3d.geometry.PointCloud()
        cloud_pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))

        mesh_pcd, mesh_down, mesh_fpfh = self._preprocess(mesh_pcd, for_mesh=True)
        cloud_pcd, cloud_down, cloud_fpfh = self._preprocess(cloud_pcd, for_mesh=False)

        init_T = self._estimate_pose_pca(cloud_down)

        if bool(self._cfg.get("use_ransac", False)):
            ransac_T = self._coarse_align_ransac(cloud_down, mesh_down, cloud_fpfh, mesh_fpfh)
            # Compose: first centroid translation, then RANSAC refinement
            init_T = ransac_T @ init_T

        icp_result = self._refine_icp(cloud_down, mesh_down, init_T)


        T_mesh_to_cloud = icp_result.transformation.astype(np.float32)
        return RegistrationResult(
            T_mesh_to_cloud=T_mesh_to_cloud,
            fitness=float(icp_result.fitness),
            inlier_rmse=float(icp_result.inlier_rmse),
        )

    def visualize_alignment(
        self,
        object_point_cloud: np.ndarray,
        mesh_path: str | Path,
        T_mesh_to_cloud: np.ndarray,
    ) -> None:
        """
        Visualize alignment in Open3D.

        Args:
            object_point_cloud (np.ndarray): (N, 3) observed point cloud in meters.
            mesh_path (str | Path): Path to mesh.
            T_mesh_to_cloud (np.ndarray): (4, 4) transform mapping mesh -> cloud frame.
        """
        pts = np.asarray(object_point_cloud, dtype=np.float32)
        mesh = self._load_mesh(mesh_path)
        mesh_pcd = self._mesh_to_point_cloud(mesh)

        cloud_pcd = o3d.geometry.PointCloud()
        cloud_pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))

        mesh_pcd.paint_uniform_color([0.0, 0.7, 0.0])
        cloud_pcd.paint_uniform_color([0.8, 0.0, 0.0])

        mesh_pcd.transform(np.asarray(T_mesh_to_cloud, dtype=np.float64))
        o3d.visualization.draw_geometries([mesh_pcd, cloud_pcd])

    # -------------------- internal helpers --------------------

    def _load_mesh(self, mesh_path: str | Path) -> "o3d.geometry.TriangleMesh":
        path = Path(mesh_path).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"Mesh file not found: {path}")

        mesh = o3d.io.read_triangle_mesh(str(path))
        if mesh.is_empty():
            raise ValueError(f"Failed to load mesh: {path}")

        # 1) Scale mesh units (mm -> m)
        scale = float(self._cfg.get("mesh_unit_scale", 1.0))
        if scale != 1.0:
            mesh.scale(scale, center=mesh.get_center())

        # 2) NEW: center the mesh at the origin to define a stable "mesh frame"
        # CAD exports sometimes include a huge global offset; ICP will fail unless we remove it.
        mesh_center = np.asarray(mesh.get_center(), dtype=np.float64)
        mesh.translate(-mesh_center)

        return mesh

    def _mesh_to_point_cloud(self, mesh: "o3d.geometry.TriangleMesh") -> "o3d.geometry.PointCloud":
        """
        Sample a triangle mesh to a point cloud.

        Returns:
            (o3d.geometry.PointCloud): Sampled point cloud in meters.
        """
        # Use Poisson disk sampling for more uniform distribution (works well for ICP).
        n_points = 4000
        try:
            pcd = mesh.sample_points_poisson_disk(number_of_points=n_points)
        except Exception:
            pcd = mesh.sample_points_uniformly(number_of_points=n_points)
        return pcd

    def _preprocess(
        self,
        pcd: "o3d.geometry.PointCloud",
        *,
        for_mesh: bool,
    ) -> tuple["o3d.geometry.PointCloud", "o3d.geometry.PointCloud", "o3d.pipelines.registration.Feature"]:
        """
        Preprocess point cloud for registration.

        Args:
            pcd (o3d.geometry.PointCloud): Input point cloud.
            for_mesh (bool): Whether this is the mesh-derived cloud (used only for naming clarity).

        Returns:
            (o3d.geometry.PointCloud): Cleaned full-res cloud.
            (o3d.geometry.PointCloud): Downsampled cloud.
            (o3d.pipelines.registration.Feature): FPFH feature on downsampled cloud.
        """
        voxel = float(self._cfg.get("voxel_size", 0.01))
        pcd_clean = pcd

        if not for_mesh:
            if bool(self._cfg.get("use_plane_removal", True)):
                dist = float(self._cfg.get("plane_distance_threshold", 0.01))
                ransac_n = int(self._cfg.get("plane_ransac_n", 3))
                iters = int(self._cfg.get("plane_num_iterations", 1000))
                num_planes = int(self._cfg.get("plane_max_planes", 2))

                for _ in range(num_planes):
                    if len(pcd_clean.points) <= 50:
                        break
                    _, inliers = pcd_clean.segment_plane(
                        distance_threshold=dist,
                        ransac_n=ransac_n,
                        num_iterations=iters,
                    )
                    if len(inliers) == 0:
                        break
                    pcd_clean = pcd_clean.select_by_index(inliers, invert=True)

            # Only clean observed cloud by default (background leakage).
            if bool(self._cfg.get("use_statistical_outlier_removal", True)):
                nb = int(self._cfg.get("sor_nb_neighbors", 30))
                std = float(self._cfg.get("sor_std_ratio", 2.0))
                pcd_clean, _ = pcd_clean.remove_statistical_outlier(nb_neighbors=nb, std_ratio=std)

            if bool(self._cfg.get("use_radius_outlier_removal", False)):
                nbp = int(self._cfg.get("ror_nb_points", 12))
                rad = float(self._cfg.get("ror_radius", 0.03))
                pcd_clean, _ = pcd_clean.remove_radius_outlier(nb_points=nbp, radius=rad)

        pcd_down = pcd_clean.voxel_down_sample(voxel)

        # Keep only the largest cluster for observed clouds
        if (not for_mesh) and bool(self._cfg.get("use_largest_cluster", False)):
            eps = float(self._cfg.get("cluster_eps", 0.03))
            min_points = int(self._cfg.get("cluster_min_points", 5))

            labels = np.array(pcd_down.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
            if labels.size > 0:
                valid = labels >= 0
                if np.any(valid):
                    # Select best cluster by size similarity to mesh
                    unique = np.unique(labels[labels >= 0])
                    if unique.size > 0:
                        # Expected mesh size after scaling (meters). Hardcode from config if you prefer.
                        # Here we read mesh size from config to keep it flexible.
                        expected = np.array(self._cfg.get("expected_object_extent_m", [0.06, 0.06, 0.08]), dtype=np.float64)

                        best_id = None
                        best_score = np.inf

                        for cid in unique:
                            idx = np.where(labels == cid)[0]
                            if idx.size < min_points:
                                continue

                            cluster = pcd_down.select_by_index(idx)
                            bbox = cluster.get_axis_aligned_bounding_box()
                            ext = np.asarray(bbox.get_extent(), dtype=np.float64)

                            if np.max(ext) > float(self._cfg.get("cluster_max_extent_m", 0.20)):
                                continue

                            # Score: how close the cluster bbox extent is to expected object extent
                            expected = np.array(
                                self._cfg.get("expected_object_extent_m", [0.06, 0.06, 0.08]),
                                dtype=np.float64,
                            )
                            score = np.linalg.norm(np.sort(ext) - np.sort(expected))

                            if score < best_score:
                                best_score = score
                                best_id = int(cid)

                            # Score: how close the cluster bbox extent is to expected object extent (order-invariant)
                            # Sort extents to ignore axis permutations.
                            score = np.linalg.norm(np.sort(ext) - np.sort(expected))

                            # Prefer tighter clusters (penalize huge extents)
                            if np.any(ext > 0.40):  # 40cm is definitely not a cup
                                score += 10.0

                            if score < best_score:
                                best_score = score
                                best_id = int(cid)

                        # Fallback: if nothing matched, pick largest cluster
                        if best_id is None:
                            unique2, counts = np.unique(labels[labels >= 0], return_counts=True)
                            best_id = int(unique2[np.argmax(counts)])

                        idx_best = np.where(labels == best_id)[0]
                        pcd_down = pcd_down.select_by_index(idx_best)
                        bbox = pcd_down.get_axis_aligned_bounding_box()
                        ext = np.asarray(bbox.get_extent(), dtype=np.float64)
                        print(f"[cluster] chosen_extent={ext}, n_points={len(pcd_down.points)}")

        normal_radius = float(self._cfg.get("normal_radius", voxel * 3.0))
        normal_max_nn = int(self._cfg.get("normal_max_nn", 30))
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=normal_radius, max_nn=normal_max_nn)
        )

        fpfh_radius = float(self._cfg.get("fpfh_radius", voxel * 5.0))
        fpfh_max_nn = int(self._cfg.get("fpfh_max_nn", 100))
        fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=fpfh_radius, max_nn=fpfh_max_nn),
        )

        return pcd_clean, pcd_down, fpfh

    def _coarse_align_ransac(
        self,
        cloud_down: "o3d.geometry.PointCloud",
        mesh_down: "o3d.geometry.PointCloud",
        cloud_fpfh: "o3d.pipelines.registration.Feature",
        mesh_fpfh: "o3d.pipelines.registration.Feature",
    ) -> np.ndarray:
        """
        Coarse alignment using RANSAC on FPFH features.

        Returns:
            (np.ndarray): (4, 4) initial transform.
        """
        dist = float(self._cfg.get("ransac_distance_threshold", 0.03))
        max_iter = int(self._cfg.get("ransac_max_iteration", 200000))
        conf = float(self._cfg.get("ransac_confidence", 0.999))

        checker1 = o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9)
        checker2 = o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(dist)

        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source=mesh_down,          # mesh -> cloud
            target=cloud_down,
            source_feature=mesh_fpfh,
            target_feature=cloud_fpfh,
            mutual_filter=True,
            max_correspondence_distance=dist,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            ransac_n=4,
            checkers=[checker1, checker2],
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(max_iter, int(max_iter * (1.0 - conf))),
        )
        return result.transformation

    def _refine_icp(
        self,
        cloud_down: "o3d.geometry.PointCloud",
        mesh_down: "o3d.geometry.PointCloud",
        init_T: np.ndarray,
    ) -> "o3d.pipelines.registration.RegistrationResult":
        """
        ICP refinement in multiple stages (coarse -> fine) to improve convergence.

        Returns:
            (o3d.pipelines.registration.RegistrationResult): ICP result.
        """
        max_iter = int(self._cfg.get("icp_max_iteration", 80))

        # Ensure normals exist for point-to-plane.
        if not mesh_down.has_normals():
            mesh_down.estimate_normals()
        if not cloud_down.has_normals():
            cloud_down.estimate_normals()

        stages = self._cfg.get("icp_stages")
        if stages is None:
            stages = [float(self._cfg.get("icp_distance_threshold", 0.10))]

        T = np.asarray(init_T, dtype=np.float64)
        result = None

        for dist in stages:
            criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
            result = o3d.pipelines.registration.registration_icp(
                source=mesh_down,  # mesh -> cloud
                target=cloud_down,
                max_correspondence_distance=float(dist),
                init=T,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                criteria=criteria,
            )
            T = result.transformation

        return result

    def _estimate_pose_pca(
        self,
        cloud: "o3d.geometry.PointCloud",
    ) -> np.ndarray:
        """
        Estimate an initial pose using PCA on the observed point cloud.

        Args:
            cloud (o3d.geometry.PointCloud): Observed object point cloud.

        Returns:
            (np.ndarray): (4, 4) initial transform from mesh frame to cloud frame.
        """
        pts = np.asarray(cloud.points, dtype=np.float64)
        if pts.shape[0] < 10:
            return np.eye(4, dtype=np.float64)

        centroid = pts.mean(axis=0)

        # PCA
        pts_centered = pts - centroid
        cov = np.cov(pts_centered.T)
        eigvals, eigvecs = np.linalg.eigh(cov)

        # principal axis = largest eigenvalue
        order = np.argsort(eigvals)[::-1]
        R = eigvecs[:, order]

        # Ensure right-handed coordinate system
        if np.linalg.det(R) < 0:
            R[:, 2] *= -1

        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = centroid

        return T
