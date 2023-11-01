import numpy as np
import pyvista as pv


def compute_draft(weight, water_density, length, width):
    return ((weight/water_density)/(length*width)) * 3


def project_mesh_to_plane(mesh, normal):
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

    for i in range(len(area_list)):
        if i == len(area_list)-1:
            f.write(f"{area_list[i]}")
        else:
            f.write(f"{area_list[i]}\n")
    f.close()


def create_angle_table(num_of_angles, plot=False):
    area_list = []
    angle_list = []
    i = 1
    step_size = 2*np.pi / num_of_angles

    for a in np.arange(0, 2 * np.pi, step_size):
        projected = project_mesh_to_plane(clipped, normal=angle_to_vector(a))

        if plot:
            p = pv.Plotter()
            p.add_text(f"{i} out of {len(np.arange(0, 2 * np.pi, step_size))}\n"
                       f"Area: {projected.area:.3f} m^2")
            p.add_mesh(poly, style='wireframe')
            p.add_mesh(clipped)
            p.add_mesh(projected, color="red")
            p.add_points(np.array(poly.center), render_points_as_spheres=True, point_size=10, color='red')
            p.add_points(np.array([0.0, 0.0, 0.0]), render_points_as_spheres=True, point_size=10, color='yellow')
            p.show()
            i = i + 1

        angle_list.append(a)
        area_list.append(projected.area)
    return angle_list, area_list


if __name__ == '__main__':

    poly = pv.read('models/boat/meshes/boat3.stl')
    # poly = pv.read('models/triangle_platform/meshes/triangle_platform.stl')

    model_height = 1.5
    water_current_surface = True

    draft = compute_draft(800, 1025, 4.28, 2)
    clipped = poly.clip('z', value=-(model_height/2)+draft, invert=water_current_surface)
    angle_list, area_list = create_angle_table(256, plot=False)

    if water_current_surface:
        filename = "current_surface.txt"
    else:
        filename = "wind_surface.txt"

    write_file(filename, angle_list, area_list)




