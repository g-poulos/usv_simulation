from project_model import create_surface_angle_file
from project_model import compute_submerged_volume
from torque_dist import get_torque_dist


def main():
    stl_file = "../models/vereniki/meshes/vereniki_scaled2.stl"
    model_height = 0.95
    draft = 0.44

    create_surface_angle_file(stl_file, model_height, draft, submerged_surface=True)
    create_surface_angle_file(stl_file, model_height, draft, submerged_surface=False)

    print(f"Submerged Volume: {compute_submerged_volume(stl_file, model_height, draft)} m^3")
    print(f"Torque Vector for submerged part: {get_torque_dist(stl_file, model_height, draft, submerged_part=True)}")
    print(f"Torque Vector for surface part:   {get_torque_dist(stl_file, model_height, draft, submerged_part=False)}")


if __name__ == '__main__':
    main()


