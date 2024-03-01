import numpy as np
from force_torque_area import create_force_table, write_list_to_file
import pyvista as pv
import multiprocessing
import time


def calculate_force_torque_table(mesh, num_of_angles, num_of_threads):
    start = time.time()
    # Rotate to match gazebo orientation
    mesh.rotate_z(-90, inplace=True)

    step_size = 2*np.pi / num_of_angles
    angles = np.arange(0, 2*np.pi, step_size)

    angle_sub_lists = []
    queue_list = []
    process_list = []
    result = []

    index = 0
    for i in range(num_of_threads):
        sublist = []
        for j in range(int(num_of_angles/num_of_threads)):
            sublist.append(angles[index])
            index += 1
        angle_sub_lists.append(sublist)

    for i in range(num_of_threads):
        queue_list.append(multiprocessing.Queue())

    # Create process objects
    for i in range(num_of_threads):
        process_list.append(
            multiprocessing.Process(target=create_force_table,
                                    args=(mesh, angle_sub_lists[i], queue_list[i], i)))

    # Start the processes
    for i in range(num_of_threads):
        process_list[i].start()

    # Wait for all processes to finish
    for i in range(num_of_threads):
        process_list[i].join()

    # Retrieve results from the queues

    for i in range(num_of_threads):
        result = result + queue_list[i].get()

    end = time.time()
    print(f"Elapsed time: {(end-start)/60:.3f} mins")
    return result


if __name__ == '__main__':
    draft = 0.4520999938249588
    stl_file = "../models/vereniki/meshes/vereniki_scaled3.stl"
    num_of_angles = 128

    poly = pv.read(stl_file)
    surface_part = poly.clip_closed_surface(normal=(0, 0, 1),
                                            origin=(0, 0, poly.bounds[4]+draft))
    submerged_part = poly.clip_closed_surface(normal=(0, 0, -1),
                                              origin=(0, 0, poly.bounds[4]+draft))

    table = calculate_force_torque_table(surface_part, num_of_angles, 16)
    write_list_to_file("../models/vereniki/meshes/wind_table.csv", table)

    table = calculate_force_torque_table(submerged_part, num_of_angles, 16)
    write_list_to_file("../models/vereniki/meshes/current_table.csv", table)



