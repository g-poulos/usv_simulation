import pyvista as pv
import numpy as np
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.metrics import pairwise_distances_argmin_min
import pymeshfix


def make_manifold(mesh):
    mesh_faces = mesh.faces.reshape(-1, 4)[:, 1:]
    mesh = pymeshfix.MeshFix(mesh.points, mesh_faces)
    mesh.repair(verbose=False, joincomp=True, remove_smallest_components=False)
    return pv.PolyData(mesh.mesh)


def get_cluster_submeshes(points, labels, centers):
    distances = pairwise_distances_argmin_min(points, centers)[1]
    sub_meshes = []

    # total_volume = 0
    for i in range(4):
        max_distance_in_cluster = np.max(distances[labels == i])
        sphere_a = pv.Sphere(radius=max_distance_in_cluster, center=centers[i])
        result = sphere_a.boolean_intersection(vereniki)

        pl = pv.Plotter()
        _ = pl.add_mesh(sphere_a, color='r', style='wireframe')
        _ = pl.add_mesh(vereniki, color='b', style='wireframe')

        # print(vereniki.is_manifold)
        # print(result.is_manifold)
        result = make_manifold(result)
        _ = pl.add_mesh(result, color='lightblue')
        # print(result.is_manifold)
        # print(result.volume)
        # total_volume = total_volume + result.volume
        pl.show()
        sub_meshes.append(result)
    # print(total_volume)
    return sub_meshes


def run_kmeans(mesh, plot=False):
    points = mesh.points

    k = 4
    kmeans = KMeans(n_clusters=k)
    kmeans.fit(points)
    labels = kmeans.labels_
    centers = kmeans.cluster_centers_

    if plot:
        # Visualize the results in 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for i in range(k):
            cluster_points = points[labels == i]
            ax.scatter(cluster_points[:, 0], cluster_points[:, 1], cluster_points[:, 2], label=f'Cluster {i + 1}')

        ax.scatter(centers[:, 0], centers[:, 1], centers[:, 2], c='red', marker='x', s=400, label='Centroids')
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')
        plt.legend()
        plt.show()

    return labels, centers


if __name__ == '__main__':
    draft = 0.45
    vereniki = pv.read("../../../../gz_ws/src/usv_simulation/models/vereniki/meshes/vereniki_scaled3.stl")

    # vereniki = pv.read("../../../../gz_ws/src/usv_simulation/models/boat/meshes/boat3.stl")
    # vereniki = pv.read("../../../../Desktop/Blends/catamaran.stl")
    vereniki.plot()

    vereniki_upper_part = vereniki.clip('z', value=-((0.65 + 0.3) / 2) + draft, invert=False)
    vereniki_lower_part = vereniki.clip('z', value=-((0.65 + 0.3) / 2) + draft, invert=True)

    lbls, clstrs = run_kmeans(vereniki_lower_part, plot=True)
    sb_meshes = get_cluster_submeshes(vereniki_lower_part.points, lbls, clstrs)



