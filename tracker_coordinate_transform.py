import numpy as np

# Example calibration points
# 0,0: x=-0.242, y=-2.690
# 2,0: x=-2.179, y=-3.207
# 0,2: x=-0.773, y=-0.784
# 2,2: x=-2.767, y=-1.304

src_points = np.array([[-0.242, -2.690], [-2.179, -3.207], [-0.773, -0.784], [-2.767, -1.304]])
dst_points = np.array([[0, 0], [2, 0], [0, 2], [2, 2]])

# Prepare the matrix A and vector B
A = []
B = []
for (x, y), (x_prime, y_prime) in zip(src_points, dst_points):
    A.append([1, x, y, 0, 0, 0])
    A.append([0, 0, 0, 1, x, y])
    B.append(x_prime)
    B.append(y_prime)

A = np.array(A)
B = np.array(B)

# Solve the least squares problem
transformation_matrix, _, _, _ = np.linalg.lstsq(A, B, rcond=None)

# Reshape the transformation matrix
transformation_matrix = transformation_matrix.reshape(2, 3)
print("Transformation Matrix:")
print(transformation_matrix)

testTransform = np.dot(transformation_matrix,src_points)
print("transform test")
print(testTransform)