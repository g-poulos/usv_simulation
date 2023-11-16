import pyvista as pv
import numpy as np


def compute_draft(stl_file, mass, water_density, plot=False):
    poly = pv.read(stl_file)
    height = abs(poly.bounds[5] - poly.bounds[4])
    center = poly.center

    for i in np.arange(0.001, poly.bounds[5], 0.001):
        submerged_part = poly.clip('z', value=(-height/2)+i, invert=True)

        if submerged_part.volume >= mass/water_density:

            bounds = submerged_part.bounds
            draft = abs(bounds[5] - bounds[4])
            volume = submerged_part.volume

            if plot:
                print(f"Draft: {draft}")
                print(f"Volume: {volume}/{mass/water_density}")

                ground_plane = pv.Plane(center=[center[0], center[1], poly.bounds[4]], i_size=7, j_size=7)
                water_surface = pv.Plane(center=[center[0], center[1], submerged_part.bounds[5]], i_size=7, j_size=7)

                p = pv.Plotter()
                p.add_mesh(poly, style='wireframe')
                p.add_mesh(submerged_part)
                p.add_mesh(ground_plane, color='y')
                p.add_mesh(water_surface, color='b', opacity=0.5)
                p.show()
            return draft, volume


if __name__ == '__main__':
    vereniki = "../models/vereniki/meshes/vereniki_scaled.stl"
    boat = "../models/boat/meshes/boat3.stl"
    print(compute_draft(vereniki, 425, 1025, plot=True))
