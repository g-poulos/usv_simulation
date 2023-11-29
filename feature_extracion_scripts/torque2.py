import numpy as np
import pyvista as pv
import pymeshfix
from project_model import get_projection_area


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

        print(f"Total: {pos_offset:.4f}/{positive_range:.4f} "
              f"{neg_offset:.4f}/{-negative_range:.4f}", end='\r')
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
            p.add_points(np.array(vereniki.center_of_mass()), color='r')
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
    com = vereniki.center_of_mass()
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


if __name__ == '__main__':
    draft = 0.45
    vereniki = pv.read("../../../../gz_ws/src/usv_simulation/models/"
                       "vereniki/meshes/vereniki_scaled3.stl")
    vereniki = vereniki.clip_closed_surface(normal=(0, 0, 1),
                                            origin=(0, 0, vereniki.center_of_mass()[2]))
    com = vereniki.center_of_mass()
    vereniki.plot()

    axes = pv.Axes(show_actor=True, actor_scale=2.0, line_width=5)
    axes.origin = com

    step_size = 360 / 8
    for a in np.arange(0, 360, step_size):
        vr = vereniki.rotate_z(a, point=axes.origin, inplace=False)
        print(f"Angle {a}d: ")
        torque_part, offset = torque_point(vr, 0.1, plot=False)
        # print()
        # print(torque_part.n_points, offset)
        print()
        if torque_part:
            if torque_part.n_points:
                p = pv.Plotter()
                p.add_text(f"Angle: {a}d")
                p.add_actor(axes.actor)
                p.add_mesh(vr, style='wireframe')
                p.add_mesh(torque_part)
                p.show()
            else:
                print("Could not form mesh")
