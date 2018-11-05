import matplotlib.pyplot as plt
import numpy as np

data = [(2, 6), (3, 7), (4, 7), (5, 8), (10, 13), (12, 15), (15, 18), (18, 19), (20, 20), (30, 30), (40, 40), (50, 50), (60, 60), (70, 70), (80, 80), (90, 90), (100, 100), (120, 120), (140, 140), (145, 145), (150, 151), (160, 162)]
expected = []
actual = []
for (a, b) in data:
    expected.append(a)
    actual.append(b)


fit = np.polyfit(actual, expected, 1)
fit_fn = np.poly1d(fit)

plt.plot(actual, actual, 'ro', actual, fit_fn(actual))
plt.xlabel("Expected")
plt.ylabel("Actual")
plt.show()
