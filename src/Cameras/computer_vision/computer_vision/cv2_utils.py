"""Thin wrappers around OpenCV ArUco functions.

Centralises version-specific API calls so the rest of the package does not
need to guard against OpenCV version differences.
"""

import cv2
import numpy as np

# Mapping from human-readable name → OpenCV constant.
# Extend as needed; names are case-insensitive.
_DICT_NAME_MAP: dict[str, int] = {
    "dict_4x4_50":   cv2.aruco.DICT_4X4_50,
    "dict_4x4_100":  cv2.aruco.DICT_4X4_100,
    "dict_4x4_250":  cv2.aruco.DICT_4X4_250,
    "dict_4x4_1000": cv2.aruco.DICT_4X4_1000,
    "dict_5x5_50":   cv2.aruco.DICT_5X5_50,
    "dict_6x6_50":   cv2.aruco.DICT_6X6_50,
}


def get_aruco_dictionary(dict_id: int | str) -> cv2.aruco.Dictionary:
    """Return an ArUco dictionary.

    Parameters
    ----------
    dict_id:
        Either a ``cv2.aruco.DICT_*`` integer constant **or** a string name
        such as ``"DICT_4X4_50"`` (case-insensitive).
    """
    if isinstance(dict_id, str):
        key = dict_id.lower()
        if key not in _DICT_NAME_MAP:
            raise ValueError(
                f"Unknown ArUco dictionary name '{dict_id}'. "
                f"Valid options: {list(_DICT_NAME_MAP.keys())}"
            )
        dict_id = _DICT_NAME_MAP[key]
    return cv2.aruco.Dictionary_get(dict_id)


def create_detector_parameters() -> cv2.aruco.DetectorParameters:
    """Return a default ArUco detector parameter set."""
    return cv2.aruco.DetectorParameters_create()


def create_aruco_board(
    obj_points: np.ndarray,
    aruco_dict: cv2.aruco.Dictionary,
    ids: np.ndarray,
) -> cv2.aruco.Board:
    """Create a cv2.aruco.Board from object-space corner arrays.

    Parameters
    ----------
    obj_points:
        Float32 array of shape ``(N, 4, 3)`` — N markers, each with 4
        corners expressed in the board's object coordinate frame (metres).
    aruco_dict:
        The ArUco dictionary the markers belong to.
    ids:
        Int32 array of length N containing the marker IDs.

    Returns
    -------
    cv2.aruco.Board
    """
    # cv2.aruco.Board_create expects a Python list of (4,1,3) arrays.
    marker_corners = [
        obj_points[i].reshape(4, 1, 3).astype(np.float32)
        for i in range(len(ids))
    ]
    return cv2.aruco.Board_create(marker_corners, aruco_dict, ids)


def detect_markers(
    image: np.ndarray,
    aruco_dict: cv2.aruco.Dictionary,
    params: cv2.aruco.DetectorParameters,
):
    """Detect ArUco markers in a BGR image.

    Returns
    -------
    corners, ids, rejected_img_points
        Same contract as ``cv2.aruco.detectMarkers``.
    """
    return cv2.aruco.detectMarkers(image, aruco_dict, parameters=params)


def estimate_pose_single_markers(
    corners,
    marker_size: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
):
    """Estimate individual poses for each detected ArUco marker.

    Parameters
    ----------
    corners:
        Output of :func:`detect_markers`.
    marker_size:
        Physical side length of each marker in metres.
    camera_matrix, dist_coeffs:
        Camera intrinsics (e.g. from ``CameraInfo``).

    Returns
    -------
    rvecs : list of np.ndarray
        Rotation vectors (Rodrigues), one per marker, shape ``(1, 1, 3)``.
    tvecs : list of np.ndarray
        Translation vectors, one per marker, shape ``(1, 1, 3)``.
    """
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners, marker_size, camera_matrix, dist_coeffs
    )
    return rvecs, tvecs


def average_poses(rvecs, tvecs) -> tuple[np.ndarray, np.ndarray]:
    """Average a list of poses into a single representative pose.

    Translations are mean-averaged. Rotations are averaged via SVD projection
    onto SO(3) so the result is always a valid rotation matrix.

    Parameters
    ----------
    rvecs, tvecs:
        Lists/arrays of rotation and translation vectors from
        :func:`estimate_pose_single_markers`.

    Returns
    -------
    rvec_mean : np.ndarray, shape ``(3, 1)``
    tvec_mean : np.ndarray, shape ``(3, 1)``
    """
    tvec_mean = np.mean(
        [t.flatten() for t in tvecs], axis=0, keepdims=True
    ).reshape(3, 1)

    rot_sum = np.zeros((3, 3), dtype=np.float64)
    for rv in rvecs:
        R, _ = cv2.Rodrigues(rv.flatten())
        rot_sum += R

    U, _, Vt = np.linalg.svd(rot_sum)
    R_mean = U @ Vt
    if np.linalg.det(R_mean) < 0:  # ensure proper rotation (det = +1)
        U[:, -1] *= -1
        R_mean = U @ Vt

    rvec_mean, _ = cv2.Rodrigues(R_mean)
    return rvec_mean, tvec_mean
