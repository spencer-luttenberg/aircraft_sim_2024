import numpy as np
from matplotlib import pyplot as plt
from airplane_sim.first_airplane import Airplane


A = np.arange(-20, 20, 0.01)

my_plane = Airplane()
my_plane.print_position()

y = A ** 2 + 5
plt.title("Matplotlib demo") 
plt.xlabel("x axis caption") 
plt.ylabel("y axis caption") 
plt.plot(A,y) 
plt.show()
