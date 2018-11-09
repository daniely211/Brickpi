from statistics import mean
import matplotlib.pyplot as plt
import numpy as np
#
# def best_fit_slope_and_intercept(xs,ys):
#     m = (((np.mean(xs) * np.mean(ys)) - np.mean(xs*ys)) /
#          ((np.mean(xs) * np.mean(xs)) - np.mean(xs*xs)))
#
#     c = np.mean(ys) - m * np.mean(xs)
#
#     return m, c
#
# accurateData = [(20, 20), (30, 30), (40, 40), (50, 50), (60, 60), (70, 70), (80, 80), (90, 90), (100, 100), (120, 120), (140, 140), (145, 145)]
# dataLow = [(2, 6), (3, 7), (4, 7), (5, 8), (10, 13), (12, 15), (15, 18), (18, 19)]
# dataHigh = [(150, 151), (160, 162), (170, 175), (180, 184), (200, 210)]
#
# expectedLow = np.array([])
# actualLow = np.array([])
# expectedHigh = np.array([])
# actualHigh = np.array([])
#
# for (a, b) in dataLow:
#     expectedLow = np.append(a, expectedLow)
#     actualLow = np.append(b, actualLow)
#
# for (a, b) in dataHigh:
#     expectedHigh = np.append(a, expectedHigh)
#     actualHigh = np.append(b, actualHigh)
#
# mL, cL = best_fit_slope_and_intercept(expectedLow,actualLow)
# mH, cH = best_fit_slope_and_intercept(expectedHigh,actualHigh)


mL = 0.876923076923
cL = 4.06153846154
mH = 1.17297297297
mH = 1.17297297297

# print(actual)

# fit = np.polyfit(actualLow, expectedLow, 1)
# fit_fn = np.poly1d(fit)
#
# plt.plot(actual, expected, 'ro', actual, fit_fn(actual))
# plt.xlabel("Expected")
# plt.ylabel("Actual")
# plt.show()

def sonar(estimate):
    if (estimate > 145):
        return (mH * estimate + cH )
    elif (estimate < 20):
        return (mL * estimate + cL)
    else:
        return (estimate)
