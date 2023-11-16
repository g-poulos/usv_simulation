import numpy as np
import pyvista as pv
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


def get_torque_dist(stl_model, draft, submerged_part=True, plot=False):
    poly = pv.read(stl_model)
    height = poly.bounds[5] - poly.bounds[4]

    submerged_poly = poly.clip('z', value=-(height/2)+draft, invert=submerged_part)
    center_of_mass = poly.center
    points = submerged_poly.points

    if plot:
        bounds = submerged_poly.bounds

        submerged_poly.plot(style='wireframe')
        ax = plt.axes(projection="3d")
        ax.set_xlim(bounds[0], bounds[1])
        ax.set_ylim(bounds[2], bounds[3])
        ax.set_zlim(bounds[4], bounds[5])

        ax.scatter3D(points[:, 0], points[:, 1], points[:, 2])
        ax.scatter3D(center_of_mass[0], center_of_mass[1], center_of_mass[2])
        ax.plot3D([center_of_mass[0], points[0][0]],
                  [center_of_mass[1], points[0][1]],
                  [center_of_mass[2], points[0][2]], c='r')
        plt.show()

    distances = center_of_mass - np.array(points)
    return np.mean(distances, axis=0)


if __name__ == '__main__':
    # stl_file = "../models/boat/meshes/boat3.stl"
    # model_height = 1.5
    # draft = compute_draft(800, 1025, 4.28, 2)

    stl_file = "../models/vereniki/meshes/vereniki_scaled2.stl"
    draft = 0.44

    print(f"Torque Vector for submerged part: {get_torque_dist(stl_file, draft, submerged_part=True, plot=True)}")
    print(f"Torque Vector for surface part:   {get_torque_dist(stl_file, draft, submerged_part=False, plot=True)}")



