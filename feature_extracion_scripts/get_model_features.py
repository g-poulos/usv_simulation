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
    print(f"Submerged Volume: {volume} m^3")
    print(f"Draft: {draft} m")


if __name__ == '__main__':
    main()


