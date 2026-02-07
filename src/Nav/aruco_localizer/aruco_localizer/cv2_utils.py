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

import cv2
from packaging import version


def get_opencv_version():
    """Get the OpenCV version as a comparable version object"""
    return version.parse(cv2.__version__)


# Version breakpoint: 4.7.0 is when the API changed significantly
OPENCV_VERSION = get_opencv_version()
IS_NEW_API = OPENCV_VERSION >= version.parse("4.7.0")


def get_aruco_dictionary(dictionary_id):
    """
    Get an ArUco dictionary compatible with the current OpenCV version.

    Args:
        dictionary_id: Integer constant for the dictionary (e.g., cv2.aruco.DICT_6X6_250)

    Returns:
        ArUco dictionary object
    """
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
