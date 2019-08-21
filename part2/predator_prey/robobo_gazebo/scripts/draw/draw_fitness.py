import matplotlib.pyplot as plt
import numpy as np

def draw(filename, color, label):
    f = open(filename, 'r')        
    x = f.read().strip().split(" ")
    
    print(x)
    
    for idx, value in enumerate(x):
        print(value)
        x[idx] = float(value)
        
    plt.plot(x, color, label = label + " avg")    
    f.close()



draw('fitness/Predator0/fitness.txt', 'r', 'Predator0')
draw('fitness/Predator1/fitness.txt', 'g', 'Predator1')
draw('fitness/Predator2/fitness.txt', 'b', 'Predator2')
#draw('fitness/Prey0/fitness.txt', 'black', 'Prey0')

plt.title("Fitness")
plt.legend()
plt.ylabel("Fitness")
plt.xlabel("Ind")
plt.show()


