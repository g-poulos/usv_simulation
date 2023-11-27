import pyvista as pv
import numpy as np
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pymeshfix
from project_model import get_projection_area


def make_manifold(mesh):
    mesh_faces = mesh.faces.reshape(-1, 4)[:, 1:]
    mesh = pymeshfix.MeshFix(mesh.points, mesh_faces)
    mesh.repair(verbose=False, joincomp=True, remove_smallest_components=False)
    return pv.PolyData(mesh.mesh)


def get_cluster_submeshes(mesh, labels, centers):
    points = mesh.points
    mesh.plot_normals(mag=0.1, faces=True, show_edges=True)
    sub_meshes = []

    for i in range(len(centers)):
        distances = np.linalg.norm(points[labels == i] - centers[i], axis=1)
        max_distance_in_cluster = np.max(distances)
        sphere_a = pv.Sphere(radius=max_distance_in_cluster, center=centers[i])
        result = sphere_a.boolean_intersection(mesh)

        pl = pv.Plotter()
        _ = pl.add_mesh(sphere_a, color='r', style='wireframe')
        _ = pl.add_mesh(mesh, color='b', style='wireframe')
        _ = pl.add_mesh(result, color='lightblue')
        pl.show()
        sub_meshes.append(result)
    return sub_meshes


def run_kmeans(mesh, k=4, plot=False):
    points = mesh.points

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

    vereniki_upper_part = vereniki.clip_closed_surface('z', origin=(0, 0, vereniki.bounds[4]+draft))
    vereniki_lower_part = vereniki.clip_closed_surface('-z', origin=(0, 0, vereniki.bounds[4]+draft))

    print(vereniki_upper_part.is_manifold)
    input_mesh = vereniki_upper_part
    input_mesh.plot()

    lbls, clstrs = run_kmeans(input_mesh, k=3, plot=True)
    sb_meshes = get_cluster_submeshes(input_mesh, lbls, clstrs)

    for msh in sb_meshes:
        print(get_projection_area(msh, normal=(1, 0, 0), plot=True))



