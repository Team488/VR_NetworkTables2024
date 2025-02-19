import numpy as np

def find_affine_transform(src_points, dst_points):
    """
    Find the affine transformation matrix that maps src_points to dst_points.
    
    :param src_points: Source points in the original coordinate system.
    :param dst_points: Destination points in the new coordinate system.
    :return: 3x3 affine transformation matrix.
    """
    assert src_points.shape == dst_points.shape, "Source and destination points must have the same shape."
    
    # Create matrices for the affine transformation
    n = src_points.shape[0]
    A = np.zeros((2 * n, 6))
    B = np.zeros((2 * n))

    for i in range(n):
        A[2 * i] = [src_points[i, 0], src_points[i, 1], 1, 0, 0, 0]
        A[2 * i + 1] = [0, 0, 0, src_points[i, 0], src_points[i, 1], 1]
        B[2 * i] = dst_points[i, 0]
        B[2 * i + 1] = dst_points[i, 1]

    # Solve for the affine transformation parameters
    params, _, _, _ = np.linalg.lstsq(A, B, rcond=None)
    transform_matrix = np.array([
        [params[0], params[1], params[2]],
        [params[3], params[4], params[5]],
        [0, 0, 1]
    ])

    return transform_matrix

def apply_transform(point, transform_matrix):
    """
    Apply the affine transformation to a point.
    
    :param point: The original point (x, y) in the first coordinate system.
    :param transform_matrix: 3x3 affine transformation matrix.
    :return: Transformed point (x', y') in the second coordinate system.
    """
    original_point = np.array([point[0], point[1], 1])
    transformed_point = np.dot(transform_matrix, original_point)
    return transformed_point[:2]



def compute_transformation_matrix(source_points, target_points):
    # assert len(src_points) == len(dst_points) == 3, "Need three points for a unique solution"
    
    # A = []
    # B = []
    
    # for (src, dst) in zip(src_points, dst_points):
    #     A.append([src[0], src[1], 1, 0, 0, 0])
    #     A.append([0, 0, 0, src[0], src[1], 1])
    #     B.append(dst[0])
    #     B.append(dst[1])
    
    # A = np.array(A)
    # B = np.array(B)
    
    # # Solve the linear system
    # transform_params = np.linalg.solve(A, B)
    
    # # Create the transformation matrix
    # transformation_matrix = np.array([
    #     [transform_params[0], transform_params[1], transform_params[2]],
    #     [transform_params[3], transform_params[4], transform_params[5]],
    #     [0, 0, 1]
    # ])
    
    # return transformation_matrix
       # Add a column of ones to the source points for affine transformation
    source_points = np.hstack([source_points, np.ones((3, 1))])
    
    # Solve for the affine transformation matrix
    transformation_matrix = np.linalg.solve(source_points, target_points)
    
    # Append a row for homogeneous coordinates
    transformation_matrix = np.vstack([transformation_matrix, [0, 0, 1]])
    return transformation_matrix

def transform_point_np(point, translation=(0, 0), scale=(1, 1), rotation=0):
    x, y = point
    dx, dy = translation
    sx, sy = scale
    theta = rotation

    # Define the original coordinates (2D points)
    points = np.array([
        [x, y]
        ])

    # Define the scaling factors for x and y
    scale_x = sx
    scale_y = sy

    # Define the rotation angle (in radians)
    theta = rotation

    # Define the rotation matrix
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])

    # Define the translation vector
    translation_vector = np.array([dx, dy])

    # Apply the scaling
    scaled_points = points * np.array([scale_x, scale_y])

    # Apply the rotation
    rotated_points = np.dot(scaled_points, rotation_matrix.T)

    # Apply the translation
    transformed_points = rotated_points + translation_vector

    # Print the results
    print("Original points:\n", points)
    print("Scaled points:\n", scaled_points)
    print("Rotated points:\n", rotated_points)
    print("Transformed points:\n", transformed_points)
    return transformed_points[0]

    
def transform_point(point, translation=(0, 0), scale=(1, 1), rotation=0):
    """
    Transforms a point using translation, scaling, and rotation.

    Args:
        point (tuple): The (x, y) coordinates of the point to transform.
        translation (tuple): The (dx, dy) translation.
        scale (tuple): The (sx, sy) scaling factors.
        rotation (float): The rotation angle in degrees.

    Returns:
        tuple: The transformed (x', y') coordinates.
    """
    x, y = point
    dx, dy = translation
    sx, sy = scale
    theta = rotation

    # Translation matrix
    translation_matrix = np.array([[1, 0, dx],
                                    [0, 1, dy],
                                    [0, 0,  1]])
    
    # Scaling matrix
    scaling_matrix = np.array([[sx,  0, 0],
                                [ 0, sy, 0],
                                [ 0,  0, 1]])
    
    # Rotation matrix
    rotation_matrix = np.array([[ np.cos(theta), -np.sin(theta), 0],
                                 [ np.sin(theta),  np.cos(theta), 0],
                                 [ 0,               0,            1]])
    
    # Combine transformations: T = R * S * T
    transformation_matrix = translation_matrix @ scaling_matrix @ rotation_matrix

    # Homogeneous coordinates for the point
    point_homogeneous = np.array([x, y, 1])

    # Apply transformation
    transformed_point = transformation_matrix @ point_homogeneous

    # Return the transformed (x', y') coordinates
    return transformed_point[0], transformed_point[1]



# def transform_point(point, transformation_matrix):
#     # homogenous_point = np.array([point[0], point[1], 1])
#     # transformed_point = np.dot(transformation_matrix, homogenous_point)
#     # return transformed_point[:2]
#     point = np.array([*point, 1])  # Convert to homogeneous coordinates
#     transformed_point = np.dot(transformation_matrix, point)
#     return transformed_point[:2]  # Return the x', y' coordinates
