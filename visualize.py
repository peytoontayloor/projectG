"""
 Enter 1 (LR) or 2 (Clock) in command line to visualize path.txt 
"""
import numpy
import matplotlib.pyplot as plt
import matplotlib.patches as patch
import sys

data = numpy.loadtxt('path.txt')
fig, ax = plt.subplots()
ax.plot(data[:, 0],data[:, 1],'.-')

wait = True
while wait:
        
        envNum = input("Enter 1 (LR) or 2 (Clock) to visualize path.txt: ")
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
        # Grabbed from my project 4, ignore for now until clock implemented
        elif envNum == '2':
            print("Displaying path.txt, Pendulum robot")

            ax.set_xlim(-5, 5)
            ax.set_ylim(-10, 10) 
            ax.set_xlabel('theta')
            ax.set_ylabel('angular velocity') 
            
            wait = False
            break
        else:
            print("Invalid input. Enter 1 or 2.")

plt.savefig('visual.png')
plt.show()