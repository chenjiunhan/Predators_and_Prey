"""
(1+1)-ES with 1/5th success rule with visualization.
Visit my tutorial website for more: https://morvanzhou.github.io/tutorials/
"""
import numpy as np
import matplotlib.pyplot as plt

#DNA_BOUND = [0, 5]       # solution upper and lower bounds
N_GENERATIONS = 500000
MUT_STRENGTH = 0.01        # initial step size (dynamic mutation strength)
MUT_MAX = 0.03
I_S = 1
H_S = 50
DNA_SIZE = 2#H_S*2 + 2             # DNA (real number)
O_S = 1
pk_rate = 1/5

def forward(x, w1, w2, b1, b2):
    x = np.reshape(x, (1, 1));
    '''print(x.shape)
    print(w1.shape)
    print(w2.shape)
    print(b1.shape)
    print(b2.shape)'''
    a1 = x.dot(w1) + b1
    z1 = G(a1)
    y = z1.dot(w2) + b2
    return y


#def F(x): return np.sin(10*x)*x + np.cos(2*x)*x     # to find the maximum of this function
#def F(x): return x
#def G(x): return 1 / (1 + np.exp(- 0.2 * x))
def G(x): return abs(x) * (x > 0)
#def G(x): return np.sin(x)
#def F(x): return - (x+5) ** 3 + 5 * (x-6) * x - 2 * x - 10 + 2 * 0.01 * x ** 4 + 100*np.sin(x)
def F(x): return x ** 3 + (x-2) ** 2
#def F(x): return np.sin(2 * x) * 10
#def F(x): return x
# find non-zero fitness for selection

def get_fitness(x, target):
    global I_S
    w1 = target[0]
    w2 = target[1]
    b1 = target[2]
    b2 = target[3]
    
    #print(b1)
    target = forward(x, w1, w2, b1, b2)
    #print("Two values: ", target[0], F(x)[0])
    fitness = -(target[0] - F(x)) ** 2
    #print("Fitness", fitness)
    return fitness


def make_kid(parent):
    global H_S
    
    w_max = 3.0
    w_min = -3.0
    k0 = parent[0] + MUT_STRENGTH * (np.random.normal(0.0, 1.0, (1, H_S)))
    k0[k0>w_max] = w_max
    k0[k0<w_min] = w_min
    k1 = parent[1] + MUT_STRENGTH * (np.random.normal(0.0, 1.0, (H_S, 1)))
    k1[k1>w_max] = w_max
    k1[k1<w_min] = w_min
    b0 = parent[2] + MUT_STRENGTH * (np.random.normal(0.0, 1.0, (1, 1)))
    b0[b0>w_max] = w_max
    b0[b0<w_min] = w_min
    b1 = parent[3] + MUT_STRENGTH * (np.random.normal(0.0, 1.0, (1, 1)))
    b1[b1>w_max] = w_max
    b1[b1<w_min] = w_min
    
    k = (k0, k1, b0, b1)
    return k


def kill_bad(parent, kid):
    global MUT_STRENGTH, pk_rate

    x = (np.random.rand(1, 1) - 0.5) * 100
    fp = get_fitness(x, parent)[0]
    fk = get_fitness(x, kid)[0]
    #print(fp, fk)
    p_target = pk_rate
    if fp < fk:     # kid better than parent
        parent = kid
        ps = 1.     # kid win -> ps = 1 (successful offspring)
    else:
        ps = 0.
    
    # adjust global mutation strength
    MUT_STRENGTH *= np.exp(1/np.sqrt(DNA_SIZE+1) * (ps - p_target)/(1 - p_target))
    if MUT_STRENGTH > MUT_MAX:
        MUT_STRENGTH = MUT_MAX
    #print("MUT_S: ", MUT_STRENGTH)
    return parent


#y = np.ndarray(shape=(O_S, 1), dtype=float)
#print(x.shape, y.shape, W1.shape)
#parent = []
parent = (np.random.rand(I_S, H_S) / 10, np.random.rand(H_S, O_S) / 10, np.random.rand(1, 1), np.random.rand(1, 1))   # parent DNA
original_parent = parent
#x = np.linspace(*DNA_BOUND, 200)

for i in range(N_GENERATIONS):
    # ES part
    kid = make_kid(parent)
    #py, ky = F(parent), F(kid)       # for later plot
    parent = kill_bad(parent, kid)

    

    if i % 2000 == 0:

        average_diff = 0
        for n in range(100):
            x = (np.random.rand(1, 1) - 0.5) * 10 
            y = forward(x, parent[0], parent[1], parent[2], parent[3])
            average_diff = average_diff + abs(y[0]-F(x))
        average_diff = average_diff / 100
        print("AD: ", average_diff)
        print("MUT: ", MUT_STRENGTH)

        lin = np.linspace(-10, 10, 100)
        pred_y = []
        for x in lin:
            pred_y.append(forward(x ,parent[0], parent[1], parent[2], parent[3])[0])
        plt.clf()
        plt.plot(lin, F(lin), '-')
        plt.plot(lin, pred_y, '-')    
        plt.draw()
        plt.pause(0.001)
for n in range(100):
    x = (np.random.rand(1, 1) - 0.5) * 10 
    y = forward(x, parent[0], parent[1], parent[2], parent[3])
    print('y', y, 'F(x)', F(x))

print(parent)

#plt.ioff(); 
#plt.show();

