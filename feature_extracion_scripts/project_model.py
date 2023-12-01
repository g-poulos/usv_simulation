import numpy as np
import pyvista as pv
import os
import vtk
import time


def get_projection_area(mesh, normal=(1, 0, 0), iterations=10, alpha=0.5, plot=False):
    min_z = mesh.bounds[4]
    max_z = mesh.bounds[5]
    step = (max_z - min_z)/iterations
    total_area = 0

    pl = pv.Plotter()
    pl.add_mesh(mesh, style='wireframe')
    axes = pv.Axes(show_actor=True, actor_scale=2.0, line_width=5)
    axes.origin = mesh.center_of_mass()
    pl.add_actor(axes.actor)

    for i in range(iterations):
        # Take mesh slice
        clipped = mesh.clip('z', value=(i+1)*step, origin=(0, 0, min_z))
        clipped = clipped.clip('z', value=i*step, origin=(0, 0, min_z), invert=False)

        # Project slice to plane and remove duplicate points
        projected_points = project_points_to_plane(clipped, normal=normal).points
        projected_points = np.unique(projected_points, axis=0)

        # Generate new surface from projected points and add its area
        polydata = pv.PolyData(projected_points)
        surface = polydata.delaunay_2d(alpha=alpha)
        total_area = total_area + surface.area

        if plot:
            pl.add_mesh(surface, color='red', opacity=0.5)
    if plot:
        pl.show()
    return total_area


def project_points_to_plane(mesh, origin=None, normal=(1, 0, 0), inplace=False):
    """Project points of this mesh to a plane"""
    if not isinstance(mesh, (pv.PolyData)):
        raise TypeError('Please use surface meshes only.')

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


def create_angle_table(model, num_of_angles, plot=False):
    area_list = []
    angle_list = []
    step_size = 2*np.pi / num_of_angles

    for a in np.arange(0, 2*np.pi, step_size):
        rotated_model = model.rotate_z(a * (180/np.pi), inplace=False)
        area = get_projection_area(rotated_model, normal=(1, 0, 0), plot=plot)
        angle_list.append(a)
        area_list.append(area)
        print(f"{a:.3f}/{2*np.pi:.3f}", end='\r')
    print()
    return angle_list, area_list


def create_surface_angle_file(stl_file, draft, num_of_angles=256, submerged_surface=True):
    poly = pv.read(stl_file)
    height = abs(poly.bounds[5] - poly.bounds[4])

    clipped = poly.clip('z', value=-(height/2)+draft, invert=submerged_surface)

    print("Writing file...")
    angle_list, area_list = create_angle_table(clipped, num_of_angles, plot=False)

    parent_dir = os.path.dirname(stl_file) + "/"
    if submerged_surface:
        filename = parent_dir + "current_surface.txt"
    else:
        filename = parent_dir + "wind_surface.txt"

    write_file(filename, angle_list, area_list)
    print(f"Completed file: {filename}")


if __name__ == '__main__':
    # stl_file = "..models/boat/meshes/boat3.stl"
    # model_height = 1.5
    # draft = compute_draft(800, 1025, 4.28, 2)

    stl_file = "../models/vereniki/meshes/vereniki_scaled3.stl"
    draft = 0.44

    start = time.time()
    create_surface_angle_file(stl_file, draft, submerged_surface=True)
    create_surface_angle_file(stl_file, draft, submerged_surface=False)
    end = time.time()
    print(f"Elapsed time: {end-start:.3f}s")

