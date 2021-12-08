from __future__ import print_function, division
import math
import numpy as np
import time

from boothbot_common.utils import inverse_matrix_hard
from numpy.lib.function_base import average
import tf.transformations as tftrans
from scipy import linalg as sla


def test_numpy_inverse(matrix):
    test_times = 0
    total_time =0.0
    average_time = 0.0
    while True:
        consume_time=0.0
        test_times = test_times + 1
        angles = (np.random.random(3) - 0.5) * (2 * math.pi)
        matrix = tftrans.compose_matrix(translate=(1, 2, 3), angles=angles)
        start_time = time.time()
        t= np.linalg.inv(matrix)
        t1=inverse_matrix_hard(matrix)
        end_time = time.time()

        consume_time = end_time-start_time
        print(consume_time)
        total_time = total_time + consume_time
        average_time = total_time / test_times
        print("numpy average time is {}".format(average_time))
        print(t)
        print(t1)
        if test_times == 600:
            break
        time.sleep(0.1)

def test_scipy_inverse(matrix):
    test_times = 0
    total_time =0.0
    average_time = 0.0

    while True:
        consume_time=0.0
        test_times = test_times + 1
        angles = (np.random.random(3) - 0.5) * (2 * math.pi)
        matrix = tftrans.compose_matrix(translate=(0, 0, 0), angles=angles)
        start_time = time.time()
        sla.inv(matrix)
        end_time = time.time()

        consume_time = end_time-start_time
        print(consume_time)
        total_time = total_time + consume_time
        average_time = total_time / test_times
        print("scipy average time is {}".format(average_time))
        if test_times == 600:
            break
        time.sleep(0.1)

def test_hard_code_inverse(matrix):
    test_times = 0
    total_time =0.0
    average_time = 0.0

    while True:
        consume_time=0.0
        test_times = test_times + 1
        angles = (np.random.random(3) - 0.5) * (2 * math.pi)
        matrix = tftrans.compose_matrix(translate=(0, 0, 0), angles=angles)
        start_time = time.time()
        inverse_matrix_hard(matrix)
        end_time = time.time()

        consume_time = end_time-start_time
        print(consume_time)
        total_time = total_time + consume_time
        average_time = total_time / test_times
        print("hard code average time is {}".format(average_time))
        if test_times == 600:
            break
        time.sleep(0.1)

def test_matrix_inverse(matrix):
    test_times = 0
    total_time =0.0
    average_time = 0.0

    while True:
        angles = (np.random.random(3) - 0.5) * (2 * math.pi)
        matrix = tftrans.compose_matrix(translate=(0, 0, 0), angles=angles)
        consume_time=0.0
        test_times = test_times + 1

        start_time = time.time()
        np.mat(matrix).I
        end_time = time.time()

        consume_time = end_time-start_time
        print(consume_time)
        total_time = total_time + consume_time
        average_time = total_time / test_times
        print("np matrix inverse average time is {}".format(average_time))
        if test_times == 600:
            break
        time.sleep(0.1)

def main():
    print("Main function")
    print("Check hard coded function")
    angles = (np.random.random(3) - 0.5) * (2 * math.pi)
    t_inclination = tftrans.compose_matrix(translate=(0, 0, 0), angles=angles)
    #test_scipy_inverse(t_inclination)
    test_numpy_inverse(t_inclination)
    #test_hard_code_inverse(t_inclination)
    #test_matrix_inverse(t_inclination)
    if(sla.inv(t_inclination).all() == inverse_matrix_hard(t_inclination).all()):
        print("Hard coded no problem")

if __name__== "__main__":
    main()
