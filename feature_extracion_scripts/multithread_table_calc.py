import numpy as np
from project_model import create_force_table, write_list_to_file
import pyvista as pv
import multiprocessing
import time


def square_numbers(numbers, result_queue):
    result = [n * n for n in numbers]
    result_queue.put(result)


def cube_numbers(numbers, result_queue):
    result = [n * n * n for n in numbers]
    result_queue.put(result)


def calculate_force_table(mesh, num_of_angles, num_of_threads):
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

    return result


if __name__ == '__main__':
    draft = 0.44
    stl_file = "../models/vereniki/meshes/vereniki_scaled3.stl"
    poly = pv.read(stl_file)
    clipped = poly.clip_closed_surface(normal=(0, 0, -1),
                                       origin=(0, 0, poly.bounds[4]+draft))
    clipped.rotate_z(-90, inplace=True)
    clipped.plot()
    start = time.time()
    table = calculate_force_table(clipped, 256, 16)
    end = time.time()
    print(f"Elapsed time: {(end-start)/60:.3f} mins")

    write_list_to_file("../models/vereniki/meshes/wind_table.csv", table)



