import trimesh
import os

mesh_dir = "meshes/"

for stl in sorted(os.listdir(mesh_dir)):
    if not stl.endswith('.stl'):
        continue
    try:
        m = trimesh.load(f"{mesh_dir}/{stl}")
        m.apply_scale(0.001)
        m.density = 1150
        i = m.moment_inertia
        print(f"\n<!-- {stl} -->")
        print(f"<mass value=\"{m.mass:.9f}\"/>")
        print(f"<inertia ixx=\"{i[0][0]:.9f}\" iyy=\"{i[1][1]:.9f}\" izz=\"{i[2][2]:.9f}\" ixy=\"{i[0][1]:.9f}\" iyz=\"{i[1][2]:.9f}\" ixz=\"{i[0][2]:.9f}\"/>")
    except Exception as e:
        print(f"⚠️  {stl}: {e}")
