import numpy as np
import pyvista as pv
from pyvista import examples


def make_example_data():
    surface = examples.download_saddle_surface()
    points = examples.download_sparse_points()
    poly = surface.interpolate(points, radius=12.0)
    return poly


def project_points_to_plane(mesh, origin=None, normal=(1,0,0), inplace=False):
    """Project points of this mesh to a plane"""
    if not isinstance(mesh, (pv.PolyData)):
        raise TypeError('Please use surface meshes only.')
    import vtk
    if origin is None:
        origin = mesh.center
    if not inplace:
        mesh = mesh.copy()
    # Make plane
    normal = normal / np.linalg.norm(normal) # MUST HAVE MAGNITUDE OF 1
    plane = vtk.vtkPlane()
    plane.SetOrigin(origin)
    plane.SetNormal(normal)

    # Perform projection in place on the copied mesh
    f = lambda p: plane.ProjectPoint(p, p)
    np.apply_along_axis(f, 1, mesh.points)
    if not inplace:
        return mesh
    return


if __name__ == '__main__':
    # poly = make_example_data()
    poly = pv.read('models/boat/meshes/boat3.stl')
    # poly.plot()

    # Project that surface to a plane
    og = poly.center
    og[-1] -= poly.length / 3.
    projected = project_points_to_plane(poly, origin=og, normal=(1, 1, 0))
    print(projected.area)

    p = pv.Plotter()
    p.add_mesh(poly,)
    p.add_mesh(projected, )
    p.show()

