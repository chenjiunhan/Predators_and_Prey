import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LogNorm
from matplotlib.ticker import MaxNLocator

#Z = np.random.rand(10, 20)
#Z[9,19] = 100
#fitnesses = np.load("output/caught_matrix.npy")
#fitnesses = np.load("output/fitnesses.npy")
#fitnesses = np.load("output/prey_fitnesses.npy")
#fitnesses = np.load("data/f1/output/end_time.npy")
fitnesses = np.load("output/end_time.npy")
Z = fitnesses[0, 0:10]
ZP = 30 * 100 - Z
Zp = Z
print(np.argmax(ZP), np.argmax(Zp))
fig, ax = plt.subplots()
#ax.plot(ZP, label="Predators")
ax.plot(Zp, label="Prey")
#c = ax.pcolor(Z, cmap='RdBu', vmin=Z.min(), vmax=Z.max())
#ax.set_title('Coevolution')
ax.set_xlabel('Generations')
ax.set_ylabel('Score')
#fig.colorbar(c, ax=ax)
ax.yaxis.set_major_locator(MaxNLocator(integer=True))
ax.xaxis.set_major_locator(MaxNLocator(integer=True))
plt.legend()
plt.show()
