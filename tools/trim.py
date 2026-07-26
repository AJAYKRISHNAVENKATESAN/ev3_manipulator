# Standalone dev tool, not a ROS package — run from the repo root with
# `python3 tools/trim.py` (requires `pip install trimesh`, not a colcon dep).
import trimesh

# Load the conveyor belt mesh from the meshes folder
mesh = trimesh.load('ev3_manipulator/meshes/conveyor_belt_link_1.stl')

# 1. Get the exact dimensions of the bounding box (X, Y, Z)
box_size = mesh.bounding_box.extents
print(f'\nURDF Box Size: size="{box_size[0]:.4f} {box_size[1]:.4f} {box_size[2]:.4f}"')

# 2. Get the center point of the box relative to the mesh's origin
box_center = mesh.bounding_box.centroid
print(f'Mesh Centroid Offset (X, Y, Z): {box_center[0]:.4f}, {box_center[1]:.4f}, {box_center[2]:.4f}\n')
