import pyvista as pv
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from project_model import compute_draft


def get_torque_dist(stl_model, height, draft, plot=False):
    water_level_slice = stl_model.clip('z', value=-(height/2)+draft, invert=False)
    center_of_mass = stl_model.center
    points = water_level_slice.points

    if plot:
        bounds = water_level_slice.bounds

        water_level_slice.plot(style='wireframe')
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

    for point in points:
        dist = center_of_mass - point
    avg_dist = dist/len(points)

    return avg_dist


if __name__ == '__main__':
    poly = pv.read('../models/vereniki/meshes/vereniki_scaled2.stl')
    model_height = 0.95
    draft = 0.44
    # poly = pv.read('../models/boat/meshes/boat3.stl')
    # model_height = 1.5
    # draft = compute_draft(800, 1025, 4.28, 2)

    print(get_torque_dist(poly, model_height, draft, plot=True))



