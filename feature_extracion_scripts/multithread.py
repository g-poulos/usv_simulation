import numpy as np
import threading
from project_model import create_force_table
import pyvista as pv
import multiprocessing


def square_numbers(numbers, result_queue):
    result = [n * n for n in numbers]
    result_queue.put(result)


def cube_numbers(numbers, result_queue):
    result = [n * n * n for n in numbers]
    result_queue.put(result)


if __name__ == '__main__':
    draft = 0.44
    stl_file = "../models/vereniki/meshes/vereniki_scaled3.stl"
    poly = pv.read(stl_file)
    clipped = poly.clip_closed_surface(normal=(0, 0, 1),
                                       origin=(0, 0, poly.bounds[4]+draft))

    step_size = 2*np.pi / 64
    angles = np.arange(0, 2*np.pi, step_size)

    result_queue1 = multiprocessing.Queue()
    result_queue2 = multiprocessing.Queue()
    result_queue3 = multiprocessing.Queue()
    result_queue4 = multiprocessing.Queue()

    # Create process objects with arguments
    process1 = multiprocessing.Process(target=create_force_table, args=(clipped, angles[:16], result_queue1, 1))
    process2 = multiprocessing.Process(target=create_force_table, args=(clipped, angles[16:32], result_queue2, 2))
    process3 = multiprocessing.Process(target=create_force_table, args=(clipped, angles[32:48], result_queue3, 3))
    process4 = multiprocessing.Process(target=create_force_table, args=(clipped, angles[48:], result_queue4, 4))

    # Start the processes
    process1.start()
    process2.start()
    process3.start()
    process4.start()

    # Wait for both processes to finish
    process1.join()
    process2.join()
    process3.join()
    process4.join()

    # # Retrieve results from the queues
    result1 = result_queue1.get()
    result2 = result_queue2.get()
    result3 = result_queue3.get()
    result4 = result_queue4.get()

    print(result1)
    print(result2)
    print(result3)
    print(result4)

    # print(f"Square results: {result_square}")
    # print(f"Cube results: {result_cube}")

