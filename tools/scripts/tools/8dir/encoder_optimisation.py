import pandas as pd
import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
import os
import sys
import math
import datetime
import re

ONE_ROUND = 2 * math.pi
HOR_ENCODER_TOTAL = pow(2, 23) - 1
VER_ENCODER_TOTAL = pow(2, 23) - 1
ONE_VER_ENCODER_RADIAN = ONE_ROUND / VER_ENCODER_TOTAL / 4.8
ONE_HOR_ENCODER_RADIAN = ONE_ROUND / HOR_ENCODER_TOTAL
PERIOD = ONE_HOR_ENCODER_RADIAN
number_of_sides = 0
gs_name = []


def read_csv(encoder_path):
    global gs_name
    gs_name = []
    count_line = 0
    count_file = 0
    all_data = []
    try:
        for root, dirs, files in os.walk(encoder_path, topdown=True):
            files = [os.path.join(root, x) for x in files]
            for filename in files:
                if "GSP" in filename:
                    df = pd.read_csv(filename, header=None)
                    df = df.drop(columns=0)
                    all_data.append(df)
                    count_file += 1
                    count_line += df.shape[0]
                    gs_name.append(re.search(r"(GSP[0-9]-[0-9][0-9][0-9][0-9])", filename).group(1))

        return all_data

    except IOError:
        print("\nCannot open the file ", filename)
    except Exception as e:
        print(e)
        sys.exit(1)
    finally:
        print("How many files were read: ", count_file)
        print("How many lines were read: ", count_line)


def output_yaml(result, gs_name):
    output_mat = []

    encoder_offseted = []
    offsets = []
    encoder_raw = np.arange(0, HOR_ENCODER_TOTAL, int(HOR_ENCODER_TOTAL / 100))

    for e in encoder_raw:
        encoder = int(e)
        offset = func_sine(encoder, result[0], PERIOD, result[1], 0)
        offsets.append(int(offset))
        encoder_offseted.append(int(encoder - offset) % (HOR_ENCODER_TOTAL + 1))

    output_mat.append(list(encoder_raw))
    output_mat.append(offsets)
    output_mat.append(encoder_offseted)

    filename = str(gs_name + "_encoder_alignment_" + datetime.datetime.now().strftime('%d-%h-%y_%H-%M-%S') + ".yaml")
    with open(filename, "w") as file:
        file.write(str(output_mat))
        print("The result has been written into file: " + filename)


def group_data(all_data):
    for gs in all_data:
        direction_index = 1
        dist_start_RB = 0
        grouped_data = {}
        grouped_data = {1: [], 2: [], 3: [], 4: [], 5: [], 6: [], 7: [], 8: [], }

        for i in range(gs.shape[0]):
            if i == 0:
                dist_start_RB = float(gs.iloc[i, 0])
                grouped_data.update({direction_index: []})
                grouped_data[direction_index].append(list(gs.loc[i, :]))
                continue

            if abs(float(gs.iloc[i, 0]) - float(gs.iloc[i - 1, 0])) < 1:
                grouped_data[direction_index].append(list(gs.loc[i, :]))
                # direction_index += np.mod(i, 2)
            elif abs(dist_start_RB - float(gs.iloc[i, 0])) < 1:
                direction_index += 1
                grouped_data.update({direction_index: []})
                grouped_data[direction_index].append(list(gs.loc[i, :]))
            else:
                grouped_data[direction_index].append(list(gs.loc[i, :]))

        for direction, data in grouped_data.items():
            list_data = np.array(grouped_data[direction])
            list_data.astype(float)
            processed_data = []
            previous_data = 0
            for datax in list_data:
                same_data = []
                for datay in list_data:
                    if abs(datax[0] - datay[0]) < 1 and abs(previous_data - datax[0]) > 1:
                        same_data.append(datay)
                if len(same_data) != 0:
                    same_data = np.average(same_data, axis=0)
                    same_data = same_data.reshape(-1)
                    previous_data = same_data[0]
                    processed_data.append(same_data)

            grouped_data[direction] = np.array(processed_data)

        yield grouped_data


def add_vertical_offset(grouped_data):
    global number_of_sides
    for direction in grouped_data.keys():
        number_of_sides = grouped_data[direction].shape[0]
        for j in range(grouped_data[direction].shape[0]):
            d = grouped_data[direction][j]
            for i in range(0, int(len(d)), 3):
                grouped_data[direction][j][i] = grouped_data[direction][j][i] * \
                                                np.cos(ONE_VER_ENCODER_RADIAN * grouped_data[direction][j][i + 2])
                grouped_data[direction][j][i] += 0.05
    return grouped_data


def get_laser_measurement_data_of_side(grouped_data):
    len_sides = {}
    for direction in grouped_data.keys():
        for j in range(number_of_sides):
            name = "l" + str(j)
            d = grouped_data[direction][j - 1]
            temp = []
            for i in range(0, int(len(d) / 2) - 1, 3):
                gap = int(len(d) / 2)
                temp.append(cosines_law(d[i], d[i + gap],
                                        math.cos(PERIOD * (abs(d[i + 1] - d[i + gap + 1])))))
                # temp.append(abs(d[i + 1] - d[i + gap + 1]))
                # print(math.cos(ONE_HOR_ENCODER_RADIAN * (abs(d[i + 1] - d[i + gap + 1]))))
            len_sides.setdefault(name, []).append(temp)
    return len_sides


def get_laser_encoder_data_of_side(grouped_data):
    temp = []
    for j in range(number_of_sides):
        for direction in grouped_data.keys():
            d = grouped_data[direction][j - 1]
            for i in range(0, int(len(d) / 2) - 1, 3):
                gap = int(len(d) / 2)
                temp.append(d[i])
                temp.append(d[i + gap])
                temp.append(d[i + 1])
                temp.append(d[i + gap + 1])

    encoder_sides = tuple(temp)
    return encoder_sides


def post_check(result_x0, result_x1, groundtruth_sides):
    after_opt_sides = {}
    for direction in grouped_data.keys():
        for j in range(number_of_sides):
            name = "l" + str(j)
            d = grouped_data[direction][j - 1]
            temp = []
            for i in range(0, int(len(d) / 2) - 1, 3):
                gap = int(len(d) / 2)
                temp.append(cosines_law(d[i], d[i + gap], math.cos(
                    PERIOD * encoder_offset(d[i + 1], d[i + 1 + gap], result_x0, result_x1))))
                # print(abs(d[i + 1] - d[i + gap + 1]))
                # print(math.cos(PERIOD * (abs(d[i + 1] - d[i + gap + 1]))))
            after_opt_sides.setdefault(name, []).append(temp)

    i = 0
    for side, data in after_opt_sides.items():
        fig, ax = plt.subplots(figsize=(10, 10))
        scat = ax.scatter(np.arange(1, 25, 1).reshape(8, 3), data, s=200, facecolor='C0', edgecolor='k', label='data point applied with new curve')
        plt.plot([0, 25], [groundtruth_sides[i], groundtruth_sides[i]], color='red')
        plt.text(0, groundtruth_sides[i], "calculated ground truth")
        plt.xlabel("Direction of GS")
        plt.ylabel("Length of the side " + side)
        plt.legend()
        i += 1
    plt.show()


def optimisation_alg(encoder_sides):
    print("Before optimisation, the loss is " + str(func2D([0, 0], *encoder_sides)))
    plot_3D_figure(encoder_sides)
    mytakestep = MyTakeStep()
    minimizer_kwargs = {"args": encoder_sides}
    # result = sp.optimize.basinhopping(func2D, [0, 1], T=0.01, niter=1000, stepsize=1, minimizer_kwargs=minimizer_kwargs,
    #                                   take_step=mytakestep)
    result = sp.optimize.shgo(func2D, bounds=[(-3000, 3000), (-20, 20)], n=1000, iters=7, args=encoder_sides,
                              sampling_method="sobol")
    print("Global minimum: x = [%.4f, %.4f], f(x) = %.32f" % (result.x[0], result.x[1], result.fun))
    # print(result.xl)
    # print(result.funl)

    return result


# ================================================================================================
# DO NOT TOUCH THIS SECTION!
# ================================================================================================

def encoder_offset(e1, e2, a, b):
    delta_encoder = (e1 + func_sine(e1, a, PERIOD, b, 0)) - (e2 + func_sine(e2, a, PERIOD, b, 0))
    return abs(delta_encoder)


def func_sine(x, a, b, c, d):
    return a * np.sin(b * x - c) + d


def func2D(x, *args):
    a = x[0]
    b = x[1]
    results = []

    for i in range(0, len(args), int(len(args) / number_of_sides)):
        delta_encoder = []
        for j in range(int(len(args) / (4 * number_of_sides))):
            m1 = args[4 * j + 0 + i]
            m2 = args[4 * j + 1 + i]
            encoder_1_raw = args[4 * j + 2 + i]
            encoder_2_raw = args[4 * j + 3 + i]

            func = (encoder_1_raw + a * math.sin(PERIOD * encoder_1_raw - b)) - \
                   (encoder_2_raw + a * math.sin(PERIOD * encoder_2_raw - b))
            func = cosines_law(m1, m2, math.cos(PERIOD * abs(func)))

            delta_encoder.append(func)

        results.append(np.std(delta_encoder))

    if number_of_sides == 1:
        return np.array(results)[0]
    else:
        return np.std(results)


def func2D_plot(xaxis, yaxis, args):
    results = []

    for i in range(0, len(args), int(len(args) / number_of_sides)):
        delta_encoder = []
        for j in range(int(len(args) / (4 * number_of_sides))):
            m1 = args[4 * j + 0 + i]
            m2 = args[4 * j + 1 + i]
            encoder_1_raw = args[4 * j + 2 + i]
            encoder_2_raw = args[4 * j + 3 + i]

            func = (encoder_1_raw + func_sine(encoder_1_raw, xaxis, PERIOD, yaxis, 0)) - \
                   (encoder_2_raw + func_sine(encoder_2_raw, xaxis, PERIOD, yaxis, 0))
            func = cosines_law(m1, m2, np.cos(PERIOD * abs(func)))

            delta_encoder.append(func)

        results.append(np.std(delta_encoder, axis=0))

    if number_of_sides == 1:
        return np.array(results)[0]
    else:
        return np.std(results, axis=0)


def cosines_law(a, b, theta):
    return np.power(np.power(a, 2) + np.power(b, 2) - 2 * a * b * theta, 1 / 2)

# ==============================================================================================


def callback_print_fun(x, f, accepted):
    print("at minimum %.4f accepted %d" % (f, int(accepted)))


def plot_3D_figure(args):
    xaxis = np.arange(-2000, 2000, 2)
    yaxis = np.arange(-math.pi * 2, math.pi * 2, 0.01)
    xaxis, yaxis = np.meshgrid(xaxis, yaxis)
    results = func2D_plot(xaxis, yaxis, args)
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    surf = ax.plot_surface(xaxis, yaxis, np.array(results), cmap='jet',label='The searching space')
    surf._facecolors2d = surf._facecolor3d
    surf._edgecolors2d = surf._edgecolor3d
    plt.xlabel("amplitude")
    plt.ylabel("phase shift")
    plt.legend()
    plt.show()


def cal_groundtruth_side(len_sides):
    results = []
    for side, data in len_sides.items():
        data_array = np.array(data).reshape(-1)
        popt, pcov = sp.optimize.curve_fit(func_sine, np.arange(1, 25, 1), data_array,
                                           p0=[1, math.pi / 12, 0, data_array[0]])
        print("Calculated ground truth of side: " + side + " is " + str(popt[3]) + "m")
        plt.plot(np.arange(1, 25, 1), func_sine(np.arange(1, 25, 1), *popt), label='Guessed curve')
        plt.plot(np.arange(1, 25, 1), data_array, 'r.', ms=1, label='Measured length of the side')
        plt.xlabel("Direction of GS")
        plt.ylabel("Length of the side "+side)
        plt.legend()
        plt.show()
        results.append(popt[3])
    return results


class MyTakeStep:
    def __init__(self, stepsize=1):
        self.stepsize = stepsize
        self.rng = np.random.default_rng()

    def __call__(self, x):
        s = self.stepsize
        x[0] += self.rng.uniform(-50. * s, 50. * s)
        x[1] += self.rng.uniform(-0.001 * s, 0.001 * s)
        return x


if __name__ == "__main__":
    cwd = os.getcwd()
    all_data = read_csv(os.path.join(cwd, "eight_direction"))
    i = 0
    for grouped_data in group_data(all_data):
        print("Doing the optimisation for " + gs_name[i])
        grouped_data = add_vertical_offset(grouped_data)
        len_sides = get_laser_measurement_data_of_side(grouped_data)
        measured_data = get_laser_encoder_data_of_side(grouped_data)
        print("Optimising... this may take about 5 mins")
        result = optimisation_alg(measured_data)
        output_yaml([result.x[0], result.x[1]], gs_name[i])
        i += 1

        groundtruth_sides = cal_groundtruth_side(len_sides)
        post_check(result.x[0], result.x[1], groundtruth_sides)

    print("Done!")