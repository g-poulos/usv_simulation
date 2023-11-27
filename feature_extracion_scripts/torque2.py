import numpy as np
import pyvista as pv
from project_model import get_projection_area


def part_from_com(mesh, offset):
    com = vereniki.center_of_mass()
    cut = mesh.clip_closed_surface(normal=(-1, 0, 0), origin=(com[0]+offset, 0, 0))
    cut = cut.clip_closed_surface(normal=(1, 0, 0), origin=(com[0]-offset, 0, 0))

    part1 = cut.clip_closed_surface(normal=(1, 0, 0), origin=(com[0], 0, 0))
    part2 = cut.clip_closed_surface(normal=(-1, 0, 0), origin=(com[0], 0, 0))
    return part1, part2


if __name__ == '__main__':
    draft = 0.45
    vereniki = pv.read("../../../../gz_ws/src/usv_simulation/models/vereniki/meshes/vereniki_scaled3.stl")

    cut1, cut2 = part_from_com(vereniki, 0.3)
    print(get_projection_area(cut1, normal=(0, 1, 0), plot=True))
    print(get_projection_area(cut2, normal=(0, 1, 0), plot=True))

    p = pv.Plotter()
    p.add_mesh(vereniki, style='wireframe')
    p.add_mesh(cut1, color='r')
    p.add_mesh(cut2, color='g')
    p.add_points(np.array(vereniki.center_of_mass()), color='r')
    p.show()
