# This is a modified version of commands from http://ompl.kavrakilab.org/pathVisualization.html
# To run, make sure matplotlib and numpy are installed, then:
# python plotpath.py
# Requires solution.txt in the same directory, change as needed

from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt

data = numpy.loadtxt('solution.txt')
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(data[:,1],data[:,2],data[:,3],'.-')
plt.show()