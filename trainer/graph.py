import numpy as np
import matplotlib.pyplot as plt

f = open("fitness.txt", "r")
y_split = f.read().split(',')
y = []
for single_y in y_split:
    if single_y == '':
        continue
    y += [float(single_y)]

x = range(len(y))

f = open("fitness_prey.txt", "r")
y_split = f.read().split(',')
y_2 = []
for single_y in y_split:
    if single_y == '':
        continue
    y_2 += [float(single_y)]

x_2 = range(len(y_2))

y = np.array(y)
y = (y - y.min()) / (y.max() - y.min())
y_2 = np.array(y_2)
y_2 = (y_2 - y_2.min()) / (y_2.max() - y_2.min())

plt.title("Fitness of every child")
plt.xlabel("Generation")
plt.ylabel("Fitness")
plt.plot(x, y, '-', label = "Predators")
plt.plot(x_2, y_2, '-', label = "Prey")
plt.legend()
plt.show()

