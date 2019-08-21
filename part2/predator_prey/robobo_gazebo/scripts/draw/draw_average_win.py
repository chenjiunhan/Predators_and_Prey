import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LogNorm
from matplotlib.ticker import MaxNLocator

#Z = np.random.rand(10, 20)
#Z[9,19] = 100
#fitnesses = np.load("output/caught_matrix.npy")
#fitnesses = np.load("output/fitnesses.npy")
#fitnesses = np.load("output/prey_fitnesses.npy")

fitnesses1 = np.load("data/f1/output/end_time.npy")
Z1 = fitnesses1[0:100, 0:100].T
ZP1 = 30 * 100 - np.sum(Z1,axis = 0)
Zp1 = np.sum(Z1,axis = 1)

fitnesses2 = np.load("data/f2/output/end_time.npy")
Z2 = fitnesses2[0:100, 0:100].T
ZP2 = 30 * 100 - np.sum(Z2,axis = 0)
Zp2 = np.sum(Z2,axis = 1)

fitnesses3 = np.load("data/f3/output/end_time.npy")
Z3 = fitnesses3[0:100, 0:100].T
ZP3 = 30 * 100 - np.sum(Z3,axis = 0)
Zp3 = np.sum(Z3,axis = 1)

ZP_stack = np.vstack((ZP1, ZP2, ZP3))
Zp_stack = np.vstack((Zp1, Zp2, Zp1))

#print(np.argmax(ZP), np.argmax(Zp))
ax = plt.subplot(211)
ax2 = plt.subplot(212)
ZP_mean = ZP_stack.mean(axis=0)
ZP_std = ZP_stack.std(axis=0)
Zp_mean = Zp_stack.mean(axis=0)
Zp_std = Zp_stack.std(axis=0)
ax.plot(ZP_mean, label="Predators", color="r")
ax2.plot(Zp_mean, label="Prey", color="g")

#c = ax.pcolor(Z, cmap='RdBu', vmin=Z.min(), vmax=Z.max())
#ax.set_title('Coevolution')
ax2.set_xlabel('Generations')
ax.set_ylabel('Score')
ax2.set_ylabel('Score')
#fig.colorbar(c, ax=ax)
ax.yaxis.set_major_locator(MaxNLocator(integer=True))
ax.xaxis.set_major_locator(MaxNLocator(integer=True))
ax2.yaxis.set_major_locator(MaxNLocator(integer=True))
ax2.xaxis.set_major_locator(MaxNLocator(integer=True))


ax.fill_between(range(0,100), ZP_mean+ZP_std, ZP_mean-ZP_std, facecolor='r', alpha=0.2)
ax2.fill_between(range(0,100), Zp_mean+Zp_std, Zp_mean-Zp_std, facecolor='g', alpha=0.2)
ax.legend(loc="lower right")
ax2.legend(loc="lower right")

plt.show()
