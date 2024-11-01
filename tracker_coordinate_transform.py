import numpy as np

def get_transformation_matrix(src_points, dst_points):
    """
    Calculates the transformation matrix (translation, rotation, scaling) from a set of source and destination points in 2D. 

    Args:
        src_points: A list of source points, each as a tuple (x, y). 
        dst_points: A list of corresponding destination points, each as a tuple (x, y). 

    Returns:
        A 3x3 transformation matrix. 
    """
    
    # Convert points to numpy arrays for easier calculations
    src_pts = np.array(src_points)
    dst_pts = np.array(dst_points) 

    # Calculate centroid of source points
    centroid_src = np.mean(src_pts, axis=0) 
    
    # Calculate centroid of destination points
    centroid_dst = np.mean(dst_pts, axis=0) 

    # Translate source points to origin
    src_pts_centered = src_pts - centroid_src 
    dst_pts_centered = dst_pts - centroid_dst 

    # Calculate the scaling factor
    scale = np.linalg.norm(dst_pts_centered[0]) / np.linalg.norm(src_pts_centered[0])

    # Calculate rotation angle
    rotation_angle = np.arctan2(src_pts_centered[1, 1] * src_pts_centered[0, 0] - src_pts_centered[0, 1] * src_pts_centered[1, 0], 
                               src_pts_centered[0, 0] * src_pts_centered[1, 0] + src_pts_centered[0, 1] * src_pts_centered[1, 1]) 

    # Create rotation matrix
    rotation_matrix = np.array([
        [np.cos(rotation_angle), -np.sin(rotation_angle), 0], 
        [np.sin(rotation_angle), np.cos(rotation_angle), 0], 
        [0, 0, 1] 
    ])

    # Create scaling matrix
    scaling_matrix = np.array([
        [scale, 0, 0], 
        [0, scale, 0], 
        [0, 0, 1]
    ])

    # Create translation matrix
    translation_matrix = np.array([
        [1, 0, centroid_dst[0] - centroid_src[0]], 
        [0, 1, centroid_dst[1] - centroid_src[1]],
        [0, 0, 1] 
    ])

    # Combine transformations 
    transformation_matrix = translation_matrix @ scaling_matrix @ rotation_matrix 

    return transformation_matrix, translation_matrix, rotation_matrix, scaling_matrix 

def apply_transformation(point, matrix):
    """
    Applies a 2D transformation matrix to a 2D point.

    Args:
        point: A tuple representing the 2D point (x, y).
        matrix: A 3x3 numpy array representing the transformation matrix.

    Returns:
        The transformed 2D point (x', y').
    """

    # Convert the point to homogeneous coordinates
    homogeneous_point = np.array([point[0], point[1], 1])

    # Apply the transformation matrix
    transformed_point = matrix @ homogeneous_point

    # Convert the transformed point back to Cartesian coordinates
    return transformed_point[0] / transformed_point[2], transformed_point[1] / transformed_point[2]


# Example calibration points
# 0,0: x=-0.242, y=-2.690
# 2,0: x=-2.179, y=-3.207
# 0,2: x=-0.773, y=-0.784
# 2,2: x=-2.767, y=-1.304
src_points = [(-0.242, -2.690), (-2.179, -3.207), (-0.773, -0.784)] 
dst_points = [(0, 0), (2, 0), (0, 2)]
transformation_matrix, translation_matrix, rotation_matrix, scaling_matrix  = get_transformation_matrix(src_points, dst_points)
print("transformation matrix")
print(transformation_matrix)
print("translation matrix")
print(translation_matrix)
print("rotation matrix")
print(rotation_matrix)
print("scaling matrix")
print(scaling_matrix)

test_xy_src = (-0.242, -2.690)
test_xy_dst = apply_transformation(test_xy_src,transformation_matrix)

#test_xy_dst = transformation_matrix @ src_points

print ("test xy dst")
print (test_xy_dst)