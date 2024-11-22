"""
 Enter 1 (LR) or in command line to visualize path.txt 
"""
import numpy
import matplotlib.pyplot as plt
import matplotlib.patches as patch
import sys

data = numpy.loadtxt('solution_path.txt')
fig, ax = plt.subplots()
ax.plot(data[:, 0],data[:, 1],'.-', color='b', label='Robot1')
ax.plot(data[:, 2],data[:, 3],'.-', color='g', label='Robot2')
ax.plot(data[:, 4],data[:, 5],'.-', color='r', label='Robot3')
ax.plot(data[:, 6],data[:, 7],'.-', color='y', label='Robot4')

# ax.plot(data[:, 8],data[:, 9],'.-', color='b', label='Robot5')
# ax.plot(data[:, 10],data[:, 11],'.-', color='g', label='Robot6')
# ax.plot(data[:, 12],data[:, 13],'.-', color='r', label='Robot7')
# ax.plot(data[:, 14],data[:, 15],'.-', color='y', label='Robot8')

ax.plot(data[:, 8],data[:, 9],'.-', color='orange', label='Robot5')
ax.plot(data[:, 10],data[:, 11],'.-', color='brown', label='Robot6')
ax.plot(data[:, 12],data[:, 13],'.-', color='black', label='Robot7')
ax.plot(data[:, 14],data[:, 15],'.-', color='m', label='Robot8')

wait = True
while wait:
        
        envNum = input("Enter 1 (LR) or 2 (Clock) to visualize path.txt,: ")
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
        elif envNum == "2":
            ax.set_xlim(0, 11)
            ax.set_ylim(0, 11)
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_aspect('equal') 
            print("Displaying path.txt, Clock robot")

            circle = patch.Circle((5, 5), 4, fill=False, linestyle='dashed')
            ax.add_patch(circle)
            wait = False
            break
        else:
            print("Invalid input. Enter 1 or 2")

plt.savefig('path.png')
plt.show()