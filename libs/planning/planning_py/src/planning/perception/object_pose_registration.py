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
            raise ImportError("open3d not found. Please install the planning package dependencies to use ICP registration.")

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

        Key change:
        - Never crash the whole demo loop just because fitness is low.
        - Still returns the best ICP result + metrics so you can inspect quality.
        """
        np.random.seed(0)
        o3d.utility.random.seed(0)

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

        if cloud_down is None or len(cloud_down.points) < 30:
            print("[register] WARNING: cloud_down too small after preprocess. Returning identity with zero fitness.")
            return RegistrationResult(
                T_mesh_to_cloud=np.eye(4, dtype=np.float32),
                fitness=0.0,
                inlier_rmse=float("inf"),
            )

        init_T = self._estimate_pose_pca(cloud_down)

        if bool(self._cfg.get("use_ransac", False)):
            ransac_T = self._coarse_align_ransac(cloud_down, mesh_down, cloud_fpfh, mesh_fpfh)
            init_T = ransac_T @ init_T

        init_candidates = self._generate_init_candidates(init_T)

        best_result = None
        best_T = None

        # Instead of hard "fitness only", use a combined score (stable across noise).
        alpha = float(self._cfg.get("icp_score_alpha", 20.0))  # rmse weight

        def score(res) -> float:
            return float(res.fitness) - alpha * float(res.inlier_rmse)

        for T0 in init_candidates:
            res = self._refine_icp(cloud_down, mesh_down, T0)
            if best_result is None or score(res) > score(best_result):
                best_result = res
                best_T = res.transformation

        if best_result is None or best_T is None:
            print("[register] WARNING: ICP returned no result. Returning identity.")
            return RegistrationResult(
                T_mesh_to_cloud=np.eye(4, dtype=np.float32),
                fitness=0.0,
                inlier_rmse=float("inf"),
            )

        # Optional hard acceptance check (but DON'T crash demo unless you want to)
        fitness_min = float(self._cfg.get("icp_fitness_min", 0.50))
        strict = bool(self._cfg.get("icp_strict_accept", False))
        if strict and float(best_result.fitness) < fitness_min:
            raise ValueError(f"ICP failed acceptance: fitness={best_result.fitness:.4f} < {fitness_min}")
        elif float(best_result.fitness) < fitness_min:
            print(f"[register] WARNING: low fitness {best_result.fitness:.4f} (< {fitness_min}). Returning anyway.")

        T_mesh_to_cloud = np.asarray(best_T, dtype=np.float64).astype(np.float32)
        return RegistrationResult(
            T_mesh_to_cloud=T_mesh_to_cloud,
            fitness=float(best_result.fitness),
            inlier_rmse=float(best_result.inlier_rmse),
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
        if bool(self._cfg.get("mesh_center_to_origin", True)):
            mesh_center = np.asarray(mesh.get_center(), dtype=np.float64)
            mesh.translate(-mesh_center)

        return mesh

    def _mesh_to_point_cloud(self, mesh: "o3d.geometry.TriangleMesh") -> "o3d.geometry.PointCloud":
        """
        Sample a triangle mesh to a point cloud.

        Returns:
            (o3d.geometry.PointCloud): Sampled point cloud in meters.
        """
        n_points = int(self._cfg.get("mesh_sample_points", 4000))
        method = str(self._cfg.get("mesh_sample_method", "uniform")).lower()

        if method == "poisson":
            # Poisson disk sampling can be stochastic; use only if you accept variability.
            pcd = mesh.sample_points_poisson_disk(number_of_points=n_points)
        else:
            # Deterministic / much more repeatable across runs.
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
                pcd_clean = self._remove_planes(pcd_clean)

            if bool(self._cfg.get("use_statistical_outlier_removal", True)):
                pcd_clean = self._remove_statistical_outliers(pcd_clean)

            if bool(self._cfg.get("use_radius_outlier_removal", False)):
                pcd_clean = self._remove_radius_outliers(pcd_clean)

        pcd_down = pcd_clean.voxel_down_sample(voxel)

        if (not for_mesh) and bool(self._cfg.get("use_largest_cluster", False)):
            pcd_down = self._keep_largest_cluster(pcd_down)

        normal_radius = float(self._cfg.get("normal_radius", voxel * 3.0))
        normal_max_nn = int(self._cfg.get("normal_max_nn", 30))
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=normal_radius, max_nn=normal_max_nn)
        )

        fpfh_radius = float(self._cfg.get("fpfh_radius", voxel * 5.0))
        fpfh_max_nn = int(self._cfg.get("fpfh_max_nn", 100))
        if len(pcd_down.points) < 10:
            # return a dummy feature (won't be used if use_ransac is False)
            dummy = o3d.pipelines.registration.Feature()
            dummy.data = o3d.utility.DoubleVector()  # minimal placeholder
            return pcd_clean, pcd_down, dummy

        if not bool(self._cfg.get("use_ransac", False)):
            dummy = o3d.pipelines.registration.Feature()
            dummy.data = o3d.utility.DoubleVector()
            return pcd_clean, pcd_down, dummy

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
    
    @staticmethod
    def _make_yaw_rotation(theta_rad: float) -> np.ndarray:
        """
        Create a rotation about +Z axis in the mesh frame.

        Args:
            theta_rad (float): Rotation angle in radians.

        Returns:
            (np.ndarray): (4, 4) homogeneous rotation transform.
        """
        c = float(np.cos(theta_rad))
        s = float(np.sin(theta_rad))
        T = np.eye(4, dtype=np.float64)
        T[0, 0] = c
        T[0, 1] = -s
        T[1, 0] = s
        T[1, 1] = c
        return T


    def _generate_init_candidates(self, init_T: np.ndarray) -> list[np.ndarray]:
        """
        Generate multiple initial guesses to improve ICP reliability on symmetric objects.

        Args:
            init_T (np.ndarray): (4, 4) base initial transform (mesh -> cloud).

        Returns:
            (list[np.ndarray]): List of (4, 4) candidate initial transforms.
        """
        num = int(self._cfg.get("icp_num_restarts", 1))
        num = max(1, num)

        # Always include the base init.
        candidates: list[np.ndarray] = [np.asarray(init_T, dtype=np.float64)]

        # For near-axis-symmetric objects (cup/cylinder), yaw is ambiguous.
        # We sample a few yaw angles and let ICP pick the best.
        if num == 1:
            return candidates

        # Spread angles evenly in [0, 2pi).
        for k in range(1, num):
            theta = 2.0 * np.pi * (k / num)
            T_yaw = self._make_yaw_rotation(theta)

            # init_T maps mesh -> cloud. To rotate the mesh in its own frame before mapping,
            # we post-multiply: init_candidate = init_T @ T_yaw
            candidates.append(candidates[0] @ T_yaw)

        return candidates
    
    def _remove_planes(self, pcd: "o3d.geometry.PointCloud") -> "o3d.geometry.PointCloud":
        """
        Remove dominant planar surfaces from a point cloud using RANSAC plane segmentation.

        Args:
            pcd (o3d.geometry.PointCloud): Input point cloud.

        Returns:
            (o3d.geometry.PointCloud): Point cloud with detected planes removed.
        """
        dist = float(self._cfg.get("plane_distance_threshold", 0.01))
        ransac_n = int(self._cfg.get("plane_ransac_n", 3))
        iters = int(self._cfg.get("plane_num_iterations", 1000))
        num_planes = int(self._cfg.get("plane_max_planes", 2))

        pcd_clean = pcd
        for _ in range(num_planes):
            if len(pcd_clean.points) <= 50:
                break
            o3d.utility.random.seed(0)
            _, inliers = pcd_clean.segment_plane(
                distance_threshold=dist,
                ransac_n=ransac_n,
                num_iterations=iters,
            )
            if len(inliers) == 0:
                break
            pcd_clean = pcd_clean.select_by_index(inliers, invert=True)
        return pcd_clean

    def _remove_statistical_outliers(self, pcd: "o3d.geometry.PointCloud") -> "o3d.geometry.PointCloud":
        """
        Remove statistical outliers from a point cloud.

        Args:
            pcd (o3d.geometry.PointCloud): Input point cloud.

        Returns:
            (o3d.geometry.PointCloud): Filtered point cloud after statistical outlier removal.
        """
        nb = int(self._cfg.get("sor_nb_neighbors", 30))
        std = float(self._cfg.get("sor_std_ratio", 2.0))
        pcd_clean, _ = pcd.remove_statistical_outlier(nb_neighbors=nb, std_ratio=std)
        return pcd_clean


    def _remove_radius_outliers(self, pcd: "o3d.geometry.PointCloud") -> "o3d.geometry.PointCloud":
        """
        Remove radius-based outliers from a point cloud.

        Args:
            pcd (o3d.geometry.PointCloud): Input point cloud.

        Returns:
            (o3d.geometry.PointCloud): Filtered point cloud after radius outlier removal.
        """
        nbp = int(self._cfg.get("ror_nb_points", 12))
        rad = float(self._cfg.get("ror_radius", 0.03))
        pcd_clean, _ = pcd.remove_radius_outlier(nb_points=nbp, radius=rad)
        return pcd_clean


    def _keep_largest_cluster(
        self,
        pcd_down: "o3d.geometry.PointCloud",
    ) -> "o3d.geometry.PointCloud":
        """
        Pick the object cluster using DBSCAN + extent prior.

        Important:
        - We ONLY return a cluster if it looks like the expected object.
        - If nothing matches, return an EMPTY point cloud (caller should reject/retry),
            instead of returning background/table blobs.
        """
        eps0 = float(self._cfg.get("cluster_eps", 0.03))
        min_points = int(self._cfg.get("cluster_min_points", 30))

        if pcd_down is None or len(pcd_down.points) == 0:
            return pcd_down

        expected = np.array(
            self._cfg.get("expected_object_extent_m", [0.06, 0.06, 0.08]),
            dtype=np.float64,
        )

        # Size bounds
        max_extent = float(self._cfg.get("cluster_max_extent_m", 0.12))
        min_extent = float(self._cfg.get("cluster_min_extent_m", 0.008))
        extent_tol = float(self._cfg.get("cluster_extent_l2_tol", 0.04))

        def extent_score(ext: np.ndarray) -> float:
            # Cup is often observed as a partial thin strip: the smallest extent is unreliable.
            e = np.sort(ext)
            ex = np.sort(expected)
            # compare middle + largest only (ignore smallest)
            return float(np.linalg.norm(e[1:] - ex[1:]))


        def run_dbscan(eps: float) -> np.ndarray:
            return np.array(
                pcd_down.cluster_dbscan(eps=float(eps), min_points=min_points, print_progress=False)
            )

        # Adaptive DBSCAN retries
        eps_list = [eps0, eps0 * 0.7, eps0 * 0.5, eps0 * 0.35, eps0 * 0.25]
        labels = None
        eps_used = None

        for eps in eps_list:
            labels_try = run_dbscan(eps)
            if labels_try.size == 0:
                continue

            valid = labels_try >= 0
            if not np.any(valid):
                continue

            unique = np.unique(labels_try[valid])
            if unique.size == 0:
                continue

            # If only one cluster and it's too big -> likely background -> retry smaller eps
            if unique.size == 1:
                cid = int(unique[0])
                idx = np.where(labels_try == cid)[0]
                cluster = pcd_down.select_by_index(idx)
                ext = np.asarray(cluster.get_axis_aligned_bounding_box().get_extent(), dtype=np.float64)
                if np.max(ext) > max_extent:
                    continue

            labels = labels_try
            eps_used = float(eps)
            break

        if labels is None:
            print("[cluster] WARNING: DBSCAN failed for all eps retries. Returning EMPTY cluster.")
            return o3d.geometry.PointCloud()

        valid = labels >= 0
        unique = np.unique(labels[valid])

        # Strict selection
        best_id = None
        best_score = np.inf
        best_ext = None

        for cid in unique:
            idx = np.where(labels == cid)[0]
            if idx.size < min_points:
                continue

            cluster = pcd_down.select_by_index(idx)
            if len(cluster.points) == 0:
                continue

            ext = np.asarray(cluster.get_axis_aligned_bounding_box().get_extent(), dtype=np.float64)
            print(f"[cluster-debug] cid={cid}, n={idx.size}, extent={ext}")

            # hard sanity checks
            if np.max(ext) > max_extent:
                continue
            if np.min(ext) < min_extent:
                continue

            sc = extent_score(ext)

            e = np.sort(ext)
            thin_ratio = e[0] / max(e[2], 1e-9)
            if thin_ratio < float(self._cfg.get("cluster_min_thin_ratio", 0.18)):
                continue

            if sc < best_score:
                best_score = sc
                best_id = int(cid)
                best_ext = ext

        # if nothing passes sanity checks -> EMPTY
        if best_id is None:
            print(
                f"[cluster] WARNING: no cluster passed sanity checks. "
                f"eps_used={eps_used:.4f}, unique={unique.size}. Returning EMPTY cluster."
            )
            return o3d.geometry.PointCloud()

        # if score too far from expected -> EMPTY
        if best_score > extent_tol:
            print(
                f"[cluster] WARNING: best cluster far from expected (score={best_score:.4f} > tol={extent_tol}). "
                f"id={best_id}, extent={best_ext}, n_points={np.sum(labels==best_id)}. Returning EMPTY cluster."
            )
            return o3d.geometry.PointCloud()

        out = pcd_down.select_by_index(np.where(labels == best_id)[0])
        print(
            f"[cluster] chosen_id={best_id}, chosen_extent={best_ext}, "
            f"score={best_score:.4f}, n_points={len(out.points)}"
        )
        return out
