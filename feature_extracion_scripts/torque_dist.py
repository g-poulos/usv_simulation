import pyvista as pv
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans


def get_torque_dist(stl_model, height, draft, plot=False):
    water_level_slice = stl_model.slice(normal='z',
                                        origin=(0, 0, -(height / 2) + draft))
    center_of_mass = stl_model.center
    points_2d = water_level_slice.points[:, :2]

    kmeans = KMeans(n_clusters=3, random_state=42)
    kmeans.fit(points_2d)
    centers = kmeans.cluster_centers_
    predicted_labels = kmeans.labels_

    if plot:
        plt.scatter(points_2d[:, 0], points_2d[:, 1], c=predicted_labels, s=5, cmap='viridis')
        plt.scatter(centers[:, 0], centers[:, 1], c='red', s=100, marker='X', label='Centers')
        plt.scatter(center_of_mass[0], center_of_mass[1], c='blue', s=100, marker='o', label='CoM')
        plt.title("K-Means Clustering")
        plt.xlabel("X-axis")
        plt.ylabel("Y-axis")
        plt.legend()
        plt.show()

    average_distance = np.array([0, 0])
    center_of_mass_2d = center_of_mass[:2]
    for cluster_center in centers:
        average_distance = average_distance + cluster_center - center_of_mass_2d
    average_distance = average_distance/len(centers)
    average_distance = np.append(average_distance, center_of_mass[2])

    return average_distance


if __name__ == '__main__':
    poly = pv.read('../models/vereniki/meshes/vereniki_scaled2.stl')
    model_height = 0.95
    draft = 0.44

    print(get_torque_dist(poly, model_height, draft))



