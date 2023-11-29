import numpy as np
import pyvista as pv
import pymeshfix
from project_model import get_projection_area


def make_manifold(mesh):
    mesh_faces = mesh.faces.reshape(-1, 4)[:, 1:]
    mesh = pymeshfix.MeshFix(mesh.points, mesh_faces)
    mesh.repair(verbose=False, joincomp=True, remove_smallest_components=False)
    return pv.PolyData(mesh.mesh)


def split_from_com(mesh, neg_offset, pos_offset):
    com = vereniki.center_of_mass()

    cut = mesh.clip_closed_surface(normal=(1, 0, 0), origin=[com[0]-neg_offset, 0, 0])
    cut = make_manifold(cut)
    cut = cut.clip_closed_surface(normal=(-1, 0, 0), origin=[com[0]+pos_offset, 0, 0])
    cut = make_manifold(cut)

    if cut.n_points:
        left_part = cut.clip_closed_surface(normal=(1, 0, 0), origin=(com[0], 0, 0))
        right_part = cut.clip_closed_surface(normal=(-1, 0, 0), origin=(com[0], 0, 0))
    else:
        return None, None

    return left_part, right_part


def torque_point(mesh):
    bounds = mesh.bounds[:2]
    com = mesh.center_of_mass()
    x_range_negative = bounds[0] + com[0]
    x_range_positive = bounds[1] + com[0]
    x_range = x_range_positive - x_range_negative
    print(x_range_negative, x_range_positive, x_range)

    neg_offset = 0.1
    pos_offset = 0.1
    negative_area_total = 0
    positive_area_total = 0
    step = 0.1

    while neg_offset < abs(x_range_negative) or pos_offset < x_range_positive:
        if negative_area_total < positive_area_total:
            neg_offset = neg_offset + step
        elif negative_area_total > positive_area_total:
            pos_offset = pos_offset + step

        positive_part, negative_part = split_from_com(mesh, neg_offset, pos_offset)

        positive_area = get_projection_area(positive_part, normal=(0, 1, 0))
        negative_area = get_projection_area(negative_part, normal=(0, 1, 0))
        positive_area_total = positive_area_total + positive_area*pos_offset
        negative_area_total = negative_area_total + negative_area*neg_offset

        print(f"Total: {negative_area_total} {positive_area_total}")

        axes = pv.Axes(show_actor=True, actor_scale=2.0, line_width=5)
        axes.origin = com
        p = pv.Plotter()
        p.set_background('grey', 'black')
        p.add_mesh(mesh, style='wireframe')
        p.add_mesh(positive_part, color='g')
        p.add_mesh(negative_part, color='r')
        p.add_text(f"Cut1: {positive_part.is_manifold}, Cut2: {negative_part.is_manifold}", color='w')
        p.add_actor(axes.actor)
        p.add_points(np.array(vereniki.center_of_mass()), color='r')
        p.show()


if __name__ == '__main__':
    draft = 0.45
    vereniki = pv.read("../../../../gz_ws/src/usv_simulation/models/vereniki/meshes/vereniki_scaled3.stl")
    vereniki = vereniki.clip_closed_surface(normal=(0, 0, 1), origin=(0, 0, vereniki.center_of_mass()[2]))

    p = pv.Plotter()
    p.add_mesh(vereniki, style='wireframe')
    p.add_points(np.array([[-1, 0, 0], [0, 0, 0]], dtype=float))
    p.show()

    torque_point(vereniki)

    # FOR DIFFERENT ANGLES
    # axes = pv.Axes(show_actor=True, actor_scale=2.0, line_width=5)
    # axes.origin = (0, 0, 0)

    # step_size = 360 / 8
    # for a in np.arange(0, 360, step_size):
    #     vr = vereniki.rotate_z(a, point=axes.origin, inplace=False)
    #
    #     rc1, rc2 = part_from_com(vr, 0.4)
    #     if rc1 and rc2:
    #         p = pv.Plotter()
    #         p.set_background('grey', 'black')
    #         p.add_mesh(vereniki, style='wireframe')
    #         p.add_mesh(vr, color='r', style='wireframe')
    #         p.add_mesh(rc1, color='g')
    #         p.add_mesh(rc2, color='b')
    #         p.add_text(f"Cut1: {rc1.is_manifold}, Cut2: {rc2.is_manifold}", color='w')
    #         p.add_actor(axes.actor)
    #         p.add_points(np.array(vereniki.center_of_mass()), color='r')
    #         p.show()
    #
    #         print(get_projection_area(rc1, normal=(0, 1, 0), plot=True))
    #         print(get_projection_area(rc2, normal=(0, 1, 0), plot=True))
    #     else:
    #         print("Empty mesh, continuing")
