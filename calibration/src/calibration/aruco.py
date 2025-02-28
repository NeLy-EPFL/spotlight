import cv2
import matplotlib.pyplot as plt


def detect_aruco_markers(image):
    """
    Detects ArUco markers in an image and returns their IDs and corner
    locations.
    
    Args:
        image (numpy.ndarray): Input image as numpy array or cv2.Mat
        
    Returns:
        tuple: (marker_ids, corner_locations)
    """
    # Convert to grayscale if the image has 3 channels
    if len(image.shape) == 3:
        raise RuntimeError("Expected grayscale image, got 3 channels")
    
    plt.imshow(image, cmap='gray')
    plt.colorbar()
    plt.savefig("test.jpg")
    # Define the ArUco dictionary (this requires OpenCV 4.7 and later;
    # older versions have a different API)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    detector = cv2.aruco.ArucoDetector(aruco_dict)
    corners, ids, rejected = detector.detectMarkers(image)
    
    # Create a list to store corner locations more readably
    corner_locations = []
    
    if ids is not None:
        # Convert corners to a more readable format
        for corner in corners:
            # Each corner has 4 points (top-left, top-right, bottom-right,
            # bottom-left).
            # Reshape to get a 4x2 array (4 points with x, y coordinates)
            corner_points = corner.reshape((4, 2))
            corner_locations.append(corner_points)
    
    return ids, corner_locations