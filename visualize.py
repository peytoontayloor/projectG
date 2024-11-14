"""
 Enter 1 (LR) or in command line to visualize path.txt 
"""
import numpy
import matplotlib.pyplot as plt
import matplotlib.patches as patch
import sys

data = numpy.loadtxt('path.txt')
fig, ax = plt.subplots()
ax.plot(data[:, 0],data[:, 1],'.-', color='b', label='Robot1')
ax.plot(data[:, 2],data[:, 3],'.-', color='g', label='Robot2')
ax.plot(data[:, 4],data[:, 5],'.-', color='r', label='Robot3')
ax.plot(data[:, 6],data[:, 7],'.-', color='y', label='Robot4')

wait = True
while wait:
        
        envNum = input("Enter 1 (LR) to visualize path.txt: ")
        if envNum == '1':
            ax.set_xlim(1, 7)  
            ax.set_ylim(1, 9)
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_aspect('equal') 
            print("Displaying path.txt, LR robot")

            # STREET ENVIRONMENT:  (x of lower left, y of lower left), width, height
            r1 = patch.Rectangle((1, 3), 2, 4)
            r2 = patch.Rectangle((5, 3), 2, 4)

            ax.add_patch(r1)
            ax.add_patch(r2)

            wait = False
            break
        else:
            print("Invalid input. Enter 1")

plt.savefig('pathvis.png')
plt.show()