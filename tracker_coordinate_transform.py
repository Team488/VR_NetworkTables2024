import numpy as np

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
