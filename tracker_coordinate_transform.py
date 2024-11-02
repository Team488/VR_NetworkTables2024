import numpy as np

def compute_transformation_matrix(src_points, dst_points):
    assert len(src_points) == len(dst_points) == 3, "Need three points for a unique solution"
    
    A = []
    B = []
    
    for (src, dst) in zip(src_points, dst_points):
        A.append([src[0], src[1], 1, 0, 0, 0])
        A.append([0, 0, 0, src[0], src[1], 1])
        B.append(dst[0])
        B.append(dst[1])
    
    A = np.array(A)
    B = np.array(B)
    
    # Solve the linear system
    transform_params = np.linalg.solve(A, B)
    
    # Create the transformation matrix
    transformation_matrix = np.array([
        [transform_params[0], transform_params[1], transform_params[2]],
        [transform_params[3], transform_params[4], transform_params[5]],
        [0, 0, 1]
    ])
    
    return transformation_matrix

def transform_point(point, transformation_matrix):
    homogenous_point = np.array([point[0], point[1], 1])
    transformed_point = np.dot(transformation_matrix, homogenous_point)
    return transformed_point[:2]