import numpy as np;
import matplotlib.pyplot as plt;

from Grid import Grid

grid=Grid(100,100,100,100)

obstacles=[]

ob1=grid.quadObs2([0,2],[8.5,1])
obstacles.append(ob1)

ob1=grid.quadObs2([-4,8],[1,2.5])
obstacles.append(ob1)

ob1=grid.quadObs2([-4,4.5],[4,2	])
obstacles.append(ob1)

ob1=grid.quadObs2([7,3.5],[2.5,11])
obstacles.append(ob1)

ob1=grid.quadObs2([3,5],[2.5,2])
obstacles.append(ob1)

ob1=grid.quadObs2([-6,-4],[3,5])
obstacles.append(ob1)

ob1=grid.quadObs2([3,-5],[2.5,5])
obstacles.append(ob1)


fig=plt.figure()
plt.axis((-10,10,-10,10))
ax = fig.add_subplot(111)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('PI RRT# example')

for i in range(len(obstacles)):
    obx=[x[0] for x in obstacles[i]]
    oby=[x[1] for x in obstacles[i]]
    if i==0:
        ax.plot(obx,oby,'.r',label="Obstacles")
    else:
        ax.plot(obx,oby,'.r')


data=np.loadtxt("states.txt", 	delimiter=',')

obx=[x[0] for x in data]
oby=[x[1] for x in data]
ax.plot(obx,oby,'.b')

for i in range(len(data)):

	ax.plot([data[i][0],data[i][2]],[data[i][1],data[i][3]],'g')

ax.plot(data[0][0],data[0][1],'.g',label="Graph")


data=np.loadtxt("solution.txt", 	delimiter=',')


obx=[x[0] for x in data]

oby=[x[1] for x in data]

ax.plot(obx,oby,'b',label="Path")

ax.legend()


plt.show()
