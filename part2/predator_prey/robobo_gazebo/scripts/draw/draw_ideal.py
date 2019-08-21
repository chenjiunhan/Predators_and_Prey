import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LogNorm
from matplotlib.ticker import MaxNLocator

#Z = np.random.rand(10, 20)
#Z[9,19] = 100
#fitnesses = np.load("output/caught_matrix.npy")
#fitnesses = np.load("output/fitnesses.npy")
#fitnesses = np.load("output/prey_fitnesses.npy")
fitnesses = np.load("output/end_time.npy")
Z = fitnesses[0:100, 0:100].T

for i in range(100):
    for j in range(100):
        if i>j:
            Z[i,j]=1
        else:
            Z[i,j]=0


print(Z.max())
fig, ax = plt.subplots()

c = ax.pcolor(Z, cmap='RdBu', vmin=Z.min(), vmax=Z.max())
#ax.set_title('Coevolution')
ax.set_xlabel('Generation of predators')
ax.set_ylabel('Generation of prey')
fig.colorbar(c, ax=ax)
ax.yaxis.set_major_locator(MaxNLocator(integer=True))
ax.xaxis.set_major_locator(MaxNLocator(integer=True))

#plt.xlabel('')
#plt.ylabel('ylabel')

plt.show()
