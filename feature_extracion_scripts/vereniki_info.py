import numpy as np
from force_torque_area import get_projection_area
import pyvista as pv


def h(h_uc, r_uc, m, r_lc, h_lc):
    return h_uc - (1 / r_uc ** 2) * ((m / (3 * np.pi * 1024)) - (r_lc ** 2) * h_lc)


def mass(h_uc, r_uc, h, r_lc, h_lc):
    return (((h_uc - h) / (1 / r_uc**2)) + (r_lc**2) * h_lc) * 3 * np.pi * 1024


if __name__ == '__main__':
    h_original = h(6.5, 2.2, 425000, 3.5, 3.0)  # 4.994418956873127
    m_original = mass(6.5, 2.2, 4.994418956873127, 3.5, 3.0)
    h_scaled = h(0.65, 0.22, 425, 0.35, 0.3)    # 0.49944189568731256
    m_scaled = mass(0.65, 0.22, 0.49944189568731256, 0.35, 0.3)
    scaled_draft = 0.65 + 0.3 - h_scaled

    print("       |  Original         |          Scaled")
    print("--------------------------------------------------")
    print(f"H     | {h_original}  | {h_scaled}")
    print(f"Draft | {6.5 + 3 - h_original}  | {scaled_draft}")
    print(f"Mass  | {m_original}           | {m_scaled}")

    vereniki = pv.read("../models/vereniki/meshes/vereniki_scaled3.stl")
    vereniki_upper_part = vereniki.clip('z', value=-((0.65+0.3)/2)+scaled_draft, invert=False)
    vereniki_lower_part = vereniki.clip('z', value=-((0.65+0.3)/2)+0.3, invert=True)

    A_t_c = get_projection_area(vereniki_lower_part, normal=(1, 0, 0), plot=False)
    A_l_c = get_projection_area(vereniki_lower_part, normal=(0, 1, 0), plot=False)
    print(f"Current Projection: At = {A_t_c},\n"
          f"                    Al = {A_l_c}")

    A_t_w = get_projection_area(vereniki_upper_part, normal=(1, 0, 0), plot=False)
    A_l_w = get_projection_area(vereniki_upper_part, normal=(0, 1, 0), plot=False)
    print(f"Wind Projection:    At = {A_t_w},\n"
          f"                    Al = {A_l_w}")


