import numpy as np
import pyvista as pv
import os
import vtk
import time
import pymeshfix


def torque_point(mesh, step, plot=False):
    """
    :param mesh: Input mesh
    :param step: Step of the algorithm
    :param plot: True to plot in every step
    :return: The part of the mesh that causes torque about the z-axis and the distance of that point
             from the center of mass
    """

    bounds = mesh.bounds[:2]
    com = mesh.center_of_mass()
    negative_range = bounds[0] + com[0]
    positive_range = bounds[1] + com[0]

    neg_offset = 0
    pos_offset = 0
    n_area_value = 0
    p_area_value = 0
    positive_part = None
    negative_part = None
    backtrack_num = 0

    while neg_offset < abs(negative_range)+step or pos_offset < positive_range+step:

        # Max offsets check if parts are equal
        if check_values(n_area_value, p_area_value, precision=2):
            if neg_offset >= abs(negative_range) and pos_offset >= positive_range:
                return None, 0

        # Negative value smaller than positive value: increment offset or backtrack
        # to find balance point
        if n_area_value < p_area_value:
            if neg_offset <= abs(negative_range):
                neg_offset = neg_offset + step
                negative_part, n_area_value = get_torque_value(mesh, -neg_offset)
            else:
                # print("Backtracking...")
                pos_offset = pos_offset - step/10
                positive_part, p_area_value = get_torque_value(mesh, pos_offset)
                if check_values(n_area_value,
                                p_area_value,
                                precision=4-backtrack_num):
                    remaining_part = mesh.clip_closed_surface(normal=(1, 0, 0),
                                                              origin=[com[0]+pos_offset, 0, 0])
                    return remaining_part, pos_offset
                backtrack_num += 1

        # Positive value smaller than negative value: increment offset or backtrack
        # to find balance point
        elif n_area_value > p_area_value:
            if pos_offset <= positive_range:
                pos_offset = pos_offset + step
                positive_part, p_area_value = get_torque_value(mesh, pos_offset)
            else:
                # print("Backtracking...")
                neg_offset = neg_offset - step/10
                negative_part, n_area_value = get_torque_value(mesh, -neg_offset)
                if check_values(n_area_value,
                                p_area_value,
                                precision=4-backtrack_num):
                    remaining_part = mesh.clip_closed_surface(normal=(-1, 0, 0),
                                                              origin=[com[0]-neg_offset, 0, 0])
                    return remaining_part, neg_offset
                backtrack_num += 1

        # Equal values: increment both offsets
        else:
            pos_offset += step
            neg_offset += step
            positive_part, p_area_value = get_torque_value(mesh, pos_offset)
            negative_part, n_area_value = get_torque_value(mesh, -neg_offset)

        # DEBUG
        # print(f"Total: {pos_offset:.4f}/{positive_range:.4f} "
        #       f"{neg_offset:.4f}/{-negative_range:.4f}", end='\r')
        # print(f"{positive_area_torque_value} {negative_area_torque_value}")
        if plot:
            axes = pv.Axes(show_actor=True, actor_scale=2.0, line_width=5)
            axes.origin = com
            p = pv.Plotter()
            p.set_background('grey', 'black')
            p.add_mesh(mesh, style='wireframe')
            p.add_mesh(positive_part, color='g')
            p.add_mesh(negative_part, color='r')
            p.add_text(f"Cut1: {positive_part.is_manifold}, Cut2: {negative_part.is_manifold}",
                       color='w')
            p.add_actor(axes.actor)
            p.add_points(np.array(mesh.center_of_mass()), color='r')
            p.show()


def check_values(val1, val2, precision=2):
    val1 = round(val1, precision)
    val2 = round(val2, precision)
    return val1 == val2


def get_torque_value(mesh, offset):
    part = get_part_from_com(mesh, offset)
    if part:
        area = get_projection_area(part, normal=(0, 1, 0))
        return part, area*abs(offset)
    return None, 0


def get_part_from_com(mesh, offset):
    com = mesh.center_of_mass()
    side = 1

    if offset < 0:
        side = side * (-1)

    cut = mesh.clip_closed_surface(normal=(side, 0, 0), origin=[com[0], 0, 0])
    if not cut.is_manifold:
        cut = make_manifold(cut)
    cut = cut.clip_closed_surface(normal=(-side, 0, 0), origin=[com[0]+offset, 0, 0])
    if not cut.is_manifold:
        cut = make_manifold(cut)

    if not cut.n_points:
        return None
    return cut


def make_manifold(mesh):
    mesh_faces = mesh.faces.reshape(-1, 4)[:, 1:]
    mesh = pymeshfix.MeshFix(mesh.points, mesh_faces)
    mesh.repair(verbose=False, joincomp=True, remove_smallest_components=False)
    return pv.PolyData(mesh.mesh)


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


def write_list_to_file(filename, force_list):
    f = open(filename, "w")
    for line in force_list:
        f.write(f"{line}\n")
    f.close()


def create_force_table(model, angles, result_queue, thread_num=0, plot=False):
    force_info = []

    i = 0
    for a in angles:
        i +=1
        rotated_model = model.rotate_z(a * (180/np.pi), inplace=False)
        area = get_projection_area(rotated_model, plot=plot)
        torque_part, offset = torque_point(rotated_model, 0.1, plot=False)
        if torque_part:
            if torque_part.n_points:
                torque_part_area = get_projection_area(torque_part)

                if plot:
                    p = pv.Plotter()
                    p.add_text(f"Angle: {a}d")
                    axes = pv.Axes(show_actor=True, actor_scale=2.0, line_width=5)
                    axes.origin = model.center_of_mass()
                    p.add_actor(axes.actor)
                    p.add_mesh(rotated_model, style='wireframe')
                    p.add_mesh(torque_part)
                    p.show()
            else:
                torque_part_area = 0
                offset = 0
        else:
            torque_part_area = 0
            offset = 0
        force_info.append(f"{a},{area},{torque_part_area},{offset}")

        print(f"{i}  /  {len(angles)}  - Thread Num : {thread_num}")
        # print(f"{a:.3f}, {area}, {torque_part_area}, {offset}")
    print(f"Thread {thread_num} finished")
    result_queue.put(force_info)

#
# def create_surface_angle_file(stl_file, draft, num_of_angles=256, submerged_surface=True):
#     poly = pv.read(stl_file)
#
#     clipped = poly.clip_closed_surface(normal=(0, 0, 1),
#                                        origin=(0, 0, poly.bounds[4]+draft))
#     print("Generating table...")
#     force_table = create_force_table(clipped, num_of_angles, plot=False)
#     print("Writing file...")
#
#     parent_dir = os.path.dirname(stl_file) + "/"
#     if submerged_surface:
#         filename = parent_dir + "current_surface.csv"
#     else:
#         filename = parent_dir + "wind_surface.csv"
#
#     write_list_to_file(filename, force_table)
#     print(f"Completed file: {filename}")


if __name__ == '__main__':
    # stl_file = "..models/boat/meshes/boat3.stl"
    # model_height = 1.5
    # draft = compute_draft(800, 1025, 4.28, 2)

    stl_file = "../models/vereniki/meshes/vereniki_scaled3.stl"
    draft = 0.44

    start = time.time()
    # create_surface_angle_file(stl_file, draft, num_of_angles=256, submerged_surface=False)
    end = time.time()
    print(f"Elapsed time: {end-start:.3f}s")

