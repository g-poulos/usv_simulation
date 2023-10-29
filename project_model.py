import numpy as np
import pyvista as pv
from pyvista import examples


def compute_draft(weight, water_density, length, width):
    return ((weight/water_density)/(length*width)) * 3


def make_example_data():
    surface = examples.download_saddle_surface()
    points = examples.download_sparse_points()
    poly = surface.interpolate(points, radius=12.0)
    return poly


def project_mesh_to_plane(mesh, normal):
    # Project that surface to a plane
    og = mesh.center
    og[-1] -= mesh.length / 3.
    return project_points_to_plane(mesh, origin=og, normal=normal)


def angle_to_vector(angle):
    vec = (np.cos(angle), np.sin(angle), 0)
    return vec


def project_points_to_plane(mesh, origin=None, normal=(1, 0, 0), inplace=False):
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


def write_file(filename, angle_list, area_list):
    f = open(filename, "w")
    f.write(f"{len(area_list)}\n")
    for angle in angle_list:
        f.write(f"{angle}\n")
    f.write("#\n")
    for area in area_list:
        if area == area_list[-1]:
            f.write(f"{area}")
        else:
            f.write(f"{area}\n")
    f.close()


if __name__ == '__main__':

    poly = pv.read('models/boat/meshes/boat3.stl')
    # poly = pv.read('models/triangle_platform/meshes/triangle_platform.stl')

    clipped = poly.clip('z', value=-(1.5/2)+0.23, invert=True)

    area_list = []
    angle_list = []
    i = 1
    for a in np.arange(0, 2*np.pi, np.pi/128):
        projected = project_mesh_to_plane(clipped, normal=angle_to_vector(a))

        # p = pv.Plotter()
        # p.add_text(f"{i} out of {len(np.arange(0, 2*np.pi, np.pi/4))}\n"
        #            f"Area: {projected.area:.3f} m^2")
        # p.add_mesh(poly, style='wireframe')
        # p.add_mesh(clipped)
        # p.add_mesh(projected, color="red")
        # p.add_points(np.array(poly.center), render_points_as_spheres=True, point_size=10, color='red')
        # p.add_points(np.array([0.0, 0.0, 0.0]), render_points_as_spheres=True, point_size=10, color='yellow')
        # p.show()
        # i = i + 1

        angle_list.append(a)
        area_list.append(projected.area)

    write_file("angles.txt", angle_list, area_list)

    # projected = project_mesh_to_plane(clipped, normal=angle_to_vector(1.39))
    # p = pv.Plotter()
    # p.add_text(f"Area: {projected.area:.3f} m^2")
    # p.add_mesh(poly, style='wireframe')
    # p.add_mesh(clipped)
    # p.add_mesh(projected, color="red")
    # p.add_points(np.array(poly.center), render_points_as_spheres=True, point_size=10, color='red')
    # p.add_points(np.array([0.0, 0.0, 0.0]), render_points_as_spheres=True, point_size=10, color='yellow')
    # p.show()



