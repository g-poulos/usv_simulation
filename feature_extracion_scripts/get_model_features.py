from project_model import create_surface_angle_file
from torque_dist import get_torque_dist
from compute_draft import compute_draft

# GENERAL INFO
# + Model origin must be at CoM
# + Model must be manifold
# + A model mesh with more vertices outputs more accurate area calculation


def main():
    stl_file = "../models/vereniki/meshes/vereniki_scaled3.stl"
    mass = 425
    water_density = 1025

    print("Calculating draft...")
    draft, volume = compute_draft(stl_file, mass, water_density, step_size=0.0001)
    print("Calculating submerged projection table:")
    create_surface_angle_file(stl_file, draft, submerged_surface=True)
    print("Calculating surface projection table:")
    create_surface_angle_file(stl_file, draft, submerged_surface=False)

    print(f"Submerged Volume: {volume} m^3")
    print(f"Draft: {draft} m")
    print(f"Torque Vector for submerged part: {get_torque_dist(stl_file, draft, submerged_part=True)}")
    print(f"Torque Vector for surface part:   {get_torque_dist(stl_file, draft, submerged_part=False)}")


if __name__ == '__main__':
    main()


