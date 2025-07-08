import numpy as np

def compose_transform(parent_transform, child_transform):
    """
    Compose two transforms.
    """
    return parent_transform @ child_transform



parent_origin_to_joint_transform = np.array([
    [1,  0,  0,   0.00],
    [0,  0,  -1, -18.50],
    [0, 1,  0,   0.00],
    [0,  0,  0,   1.00]
])

child_origin_to_joint_transform = np.array([
    [1,  0,  0,   0.00],
    [0,  0, -1, -19.00],
    [0,  1,  0,   0.00],
    [0,  0,  0,   1.00]
])

parent_origin_to_child_origin = parent_origin_to_joint_transform @ np.linalg.inv(child_origin_to_joint_transform)
print(parent_origin_to_child_origin)

# print(parent_origin_to_joint_transform @ child_origin_to_joint_transform)
# print(np.linalg.inv(parent_origin_to_joint_transform) @ parent_origin_to_child_origin)
# print(np.linalg.inv(child_origin_to_joint_transform) @ parent_origin_to_child_origin)