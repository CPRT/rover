#!/usr/bin/env python3
"""
OpenCV ArUco Compatibility Layer

This module provides a compatibility layer for OpenCV ArUco API differences
between versions 4.5.4 and 4.12.0. It detects the OpenCV version and uses
the appropriate API calls.

Key API changes between versions:
- cv2.aruco.Dictionary_get() -> cv2.aruco.getPredefinedDictionary()
- cv2.aruco.DetectorParameters_create() -> cv2.aruco.DetectorParameters()
- cv2.aruco.Board_create() -> cv2.aruco.Board()
- cv2.aruco.detectMarkers() signature changes
- cv2.aruco.drawDetectedMarkers() signature changes
- cv2.aruco.estimatePoseBoard() signature changes
- cv2.aruco.drawAxis() -> cv2.drawFrameAxes()

Author: GitHub Copilot
Version: 2025-12-07
"""
# pylint: skip-file

import cv2
import numpy as np
from packaging import version


def get_opencv_version():
    """Get the OpenCV version as a comparable version object"""
    return version.parse(cv2.__version__)


# Version breakpoint: 4.7.0 is when the API changed significantly
OPENCV_VERSION = get_opencv_version()
IS_NEW_API = OPENCV_VERSION >= version.parse("4.7.0")

# Mapping from string names to OpenCV dictionary constants (case-insensitive).
_DICT_NAME_MAP = {
    "dict_4x4_50": cv2.aruco.DICT_4X4_50,
    "dict_4x4_100": cv2.aruco.DICT_4X4_100,
    "dict_4x4_250": cv2.aruco.DICT_4X4_250,
    "dict_4x4_1000": cv2.aruco.DICT_4X4_1000,
    "dict_5x5_50": cv2.aruco.DICT_5X5_50,
    "dict_5x5_100": cv2.aruco.DICT_5X5_100,
    "dict_5x5_250": cv2.aruco.DICT_5X5_250,
    "dict_5x5_1000": cv2.aruco.DICT_5X5_1000,
    "dict_6x6_50": cv2.aruco.DICT_6X6_50,
    "dict_6x6_100": cv2.aruco.DICT_6X6_100,
    "dict_6x6_250": cv2.aruco.DICT_6X6_250,
    "dict_6x6_1000": cv2.aruco.DICT_6X6_1000,
    "dict_7x7_50": cv2.aruco.DICT_7X7_50,
    "dict_7x7_100": cv2.aruco.DICT_7X7_100,
    "dict_7x7_250": cv2.aruco.DICT_7X7_250,
    "dict_7x7_1000": cv2.aruco.DICT_7X7_1000,
    "dict_aruco_original": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


def _normalize_dictionary_id(dictionary_id):
    if isinstance(dictionary_id, str):
        key = dictionary_id.strip().lower()
        if key not in _DICT_NAME_MAP:
            raise ValueError(
                f"Unknown ArUco dictionary name '{dictionary_id}'. "
                f"Valid options: {sorted(_DICT_NAME_MAP.keys())}"
            )
        return _DICT_NAME_MAP[key]
    return dictionary_id


def get_aruco_dictionary(dictionary_id):
    """
    Get an ArUco dictionary compatible with the current OpenCV version.

    Args:
        dictionary_id: Integer constant for the dictionary (e.g., cv2.aruco.DICT_6X6_250)

    Returns:
        ArUco dictionary object
    """
    dictionary_id = _normalize_dictionary_id(dictionary_id)
    if IS_NEW_API:
        return cv2.aruco.getPredefinedDictionary(dictionary_id)
    else:
        return cv2.aruco.Dictionary_get(dictionary_id)


def create_detector_parameters():
    """
    Create ArUco detector parameters compatible with the current OpenCV version.

    Returns:
        ArUco DetectorParameters object
    """
    if IS_NEW_API:
        return cv2.aruco.DetectorParameters()
    else:
        return cv2.aruco.DetectorParameters_create()


def create_aruco_board(obj_points, dictionary, ids):
    """
    Create an ArUco board compatible with the current OpenCV version.

    Args:
        obj_points: Array of 3D object points for each marker
        dictionary: ArUco dictionary object
        ids: Array of marker IDs

    Returns:
        ArUco Board object
    """
    if IS_NEW_API:
        # In new API, Board constructor takes objPoints and dictionary and ids
        return cv2.aruco.Board(obj_points, dictionary, ids)
    else:
        return cv2.aruco.Board_create(obj_points, dictionary, ids)


def detect_markers(image, dictionary, parameters):
    """
    Detect ArUco markers in an image compatible with the current OpenCV version.

    Args:
        image: Input image
        dictionary: ArUco dictionary object
        parameters: ArUco detector parameters

    Returns:
        Tuple of (corners, ids, rejected) where:
        - corners: List of detected marker corners
        - ids: Array of detected marker IDs
        - rejected: List of rejected marker candidates
    """
    if IS_NEW_API:
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        corners, ids, rejected = detector.detectMarkers(image)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(
            image, dictionary, parameters=parameters
        )

    return corners, ids, rejected


def draw_detected_markers(image, corners, ids):
    """
    Draw detected ArUco markers on an image compatible with the current OpenCV version.

    Args:
        image: Input/output image
        corners: List of detected marker corners
        ids: Array of detected marker IDs

    Returns:
        Image with markers drawn (modifies in-place and returns)
    """
    return cv2.aruco.drawDetectedMarkers(image, corners, ids)


def estimate_pose_board(corners, ids, board, camera_matrix, dist_coeffs):
    """
    Estimate the pose of an ArUco board compatible with the current OpenCV version.

    Args:
        corners: List of detected marker corners
        ids: Array of detected marker IDs
        board: ArUco Board object
        camera_matrix: Camera intrinsic matrix
        dist_coeffs: Camera distortion coefficients

    Returns:
        Tuple of (num_markers, rvec, tvec) where:
        - num_markers: Number of markers used for pose estimation
        - rvec: Rotation vector
        - tvec: Translation vector
    """
    if IS_NEW_API:
        # New API: returns (rvec, tvec) directly, and success is determined by non-None values
        # The signature is: estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeffs[, rvec[, tvec]]) -> retval, rvec, tvec
        obj_points, img_points = board.matchImagePoints(corners, ids)

        if obj_points is None or len(obj_points) == 0:
            return 0, None, None

        success, rvec, tvec = cv2.solvePnP(
            obj_points,
            img_points,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )

        if success:
            return (
                len(obj_points) // 4,
                rvec,
                tvec,
            )  # Divide by 4 because 4 corners per marker
        else:
            return 0, None, None
    else:
        # Old API
        num_markers, rvec, tvec = cv2.aruco.estimatePoseBoard(
            corners, ids, board, camera_matrix, dist_coeffs, None, None
        )
        return num_markers, rvec, tvec


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
    try:
        if hasattr(cv2.aruco, "estimatePoseSingleMarkers"):
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, marker_size, camera_matrix, dist_coeffs
            )
            return rvecs, tvecs
    except cv2.error:
        # Fall back to a manual solvePnP implementation below.
        pass

    return _estimate_pose_single_markers_fallback(
        corners, marker_size, camera_matrix, dist_coeffs
    )


def _estimate_pose_single_markers_fallback(
    corners,
    marker_size: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
):
    """Fallback pose estimator using solvePnP for each detected marker."""
    if corners is None:
        return [], []

    obj_points = _marker_object_points(marker_size)
    rvecs = []
    tvecs = []

    if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE"):
        primary_flag = cv2.SOLVEPNP_IPPE_SQUARE
    else:
        primary_flag = cv2.SOLVEPNP_ITERATIVE

    for corner in corners:
        img_points = np.asarray(corner, dtype=np.float32).reshape(-1, 2)
        success, rvec, tvec = cv2.solvePnP(
            obj_points,
            img_points,
            camera_matrix,
            dist_coeffs,
            flags=primary_flag,
        )
        if not success and primary_flag != cv2.SOLVEPNP_ITERATIVE:
            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                img_points,
                camera_matrix,
                dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
        if success:
            rvecs.append(rvec.reshape(1, 1, 3))
            tvecs.append(tvec.reshape(1, 1, 3))

    return rvecs, tvecs


def _marker_object_points(marker_size: float) -> np.ndarray:
    """Return marker corner points centered at the marker origin."""
    half = marker_size / 2.0
    return np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float32,
    )


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
    tvec_mean = np.mean([t.flatten() for t in tvecs], axis=0, keepdims=True).reshape(
        3, 1
    )

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


def draw_axis(image, camera_matrix, dist_coeffs, rvec, tvec, length):
    """
    Draw coordinate axes on an image compatible with the current OpenCV version.

    Args:
        image: Input/output image
        camera_matrix: Camera intrinsic matrix
        dist_coeffs: Camera distortion coefficients
        rvec: Rotation vector
        tvec: Translation vector
        length: Length of the axes to draw

    Returns:
        Image with axes drawn (modifies in-place and returns)
    """
    if IS_NEW_API:
        # New API uses cv2.drawFrameAxes
        return cv2.drawFrameAxes(
            image, camera_matrix, dist_coeffs, rvec, tvec, length, 2
        )
    else:
        # Old API uses cv2.aruco.drawAxis
        return cv2.aruco.drawAxis(image, camera_matrix, dist_coeffs, rvec, tvec, length)


def get_api_info():
    """
    Get information about the OpenCV version and API being used.

    Returns:
        Dictionary with version information
    """
    return {
        "opencv_version": cv2.__version__,
        "parsed_version": str(OPENCV_VERSION),
        "using_new_api": IS_NEW_API,
        "api_version": "4.7.0+" if IS_NEW_API else "< 4.7.0",
    }
