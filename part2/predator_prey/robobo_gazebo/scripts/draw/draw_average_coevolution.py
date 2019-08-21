import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LogNorm
from matplotlib.ticker import MaxNLocator

#Z = np.random.rand(10, 20)
#Z[9,19] = 100
#fitnesses = np.load("output/caught_matrix.npy")
#fitnesses = np.load("output/fitnesses.npy")
#fitnesses = np.load("output/prey_fitnesses.npy")
fitnesses0 = np.load("data/f1/output/end_time.npy")
fitnesses1 = np.load("data/f2/output/end_time.npy")
fitnesses2 = np.load("data/f3/output/end_time.npy")

Z = (fitnesses0[0:100, 0:100].T + fitnesses1[0:100, 0:100].T + fitnesses2[0:100, 0:100].T) / 3

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
