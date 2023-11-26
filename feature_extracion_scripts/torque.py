import pyvista as pv
import numpy as np
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.metrics import pairwise_distances_argmin_min
import pymeshfix


def plot_cluster_volume(points, labels, distances):
    total_volume = 0
    for i in range(4):
        max_distance_in_cluster = np.max(distances[labels == i])

        sphere_a = pv.Sphere(radius=max_distance_in_cluster, center=centers[i])

        result = sphere_a.boolean_intersection(vereniki)
        pl = pv.Plotter()
        _ = pl.add_mesh(sphere_a, color='r', style='wireframe')
        _ = pl.add_mesh(vereniki, color='b', style='wireframe')

        print(vereniki.is_manifold)
        print(result.is_manifold)
        # result = result.triangulate().clean(tolerance=1e-10)
        f = result.faces.reshape(-1, 4)[:, 1:]
        result = pymeshfix.MeshFix(result.points, f)
        result.repair(verbose=False, joincomp=True, remove_smallest_components=False)

        result = pv.PolyData(result.mesh)
        _ = pl.add_mesh(result, color='lightblue')
        print(result.is_manifold)
        print(result.volume)
        total_volume = total_volume + result.volume
        pl.show()
    print(total_volume)


if __name__ == '__main__':
    draft = 0.45
    vereniki = pv.read("../models/vereniki/meshes/vereniki_scaled3.stl")

    # vereniki = pv.read("../models/boat/meshes/boat3.stl")
    # vereniki = pv.read("../../../../Desktop/Blends/catamaran.stl")
    vereniki.plot()

    vereniki_upper_part = vereniki.clip('z', value=-((0.65 + 0.3) / 2) + draft, invert=False)
    vereniki_lower_part = vereniki.clip('z', value=-((0.65 + 0.3) / 2) + draft, invert=True)

    points = vereniki_upper_part.points
    faces = vereniki_lower_part.faces

    sphere = pv.Sphere(radius=0.5, center=(0, 0, 0))

    # Set the number of clusters (k)
    k = 4

    # Perform k-means clustering
    kmeans = KMeans(n_clusters=k)
    kmeans.fit(points)
    labels = kmeans.labels_
    centers = kmeans.cluster_centers_

    # Visualize the results in 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the points with different colors for each cluster
    for i in range(k):
        cluster_points = points[labels == i]
        ax.scatter(cluster_points[:, 0], cluster_points[:, 1], cluster_points[:, 2], label=f'Cluster {i + 1}')

    # Plot the cluster centers
    ax.scatter(centers[:, 0], centers[:, 1], centers[:, 2], c='red', marker='x', s=400, label='Centroids')

    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')

    plt.title('K-Means Clustering in 3D')
    plt.legend()
    plt.show()

    distances = pairwise_distances_argmin_min(points, centers)[1]
    plot_cluster_volume(points, labels, distances)


