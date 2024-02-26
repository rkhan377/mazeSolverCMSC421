import matplotlib.pyplot as plt
from matplotlib import path

#point class used to associate a distance value with each coordinate
class Point:
    def __init__(self, coords):
        self.coords = coords
        self.x = coords[0]
        self.y = coords[1]
        self.distance = 1000

#calculate manhattan distance (rather than euclidean because we are not moving diagonally) given coordinate tuples
# source: https://www.statology.org/manhattan-distance-python/
def manhattan(a, b):
    return sum(abs(val1-val2) for val1, val2 in zip(a,b))

#checks if coordinate tuple is inside obstacles' figure area
def collision(p,obstacles):
    collisions = []
    for i in obstacles:
        collisions.append(i.contains_points([p])[0])
    return True in collisions

#checks if coordinate tuple is out of bounds
def outofbounds(p, xlim,ylim):
    x = p[0]
    y = p[1]
    if x < 0 or x > xlim or y < 0 or y > ylim:
        return True
    

def astar(start, goal, obstacles, xlim, ylim):
    visited = [] #keep track of all points visited
    stack = [start] #stack of points to process
    while (len(stack)>0): 
        curr = stack.pop(0)
        if not (any(x.coords == curr.coords for x in visited)): #check if current point has been visited already
            if curr.coords == goal.coords: #checks if goal has been found, if it is then end the loop
                visited.append(curr)
                break
            #check all adjacent points, if not already visited then add
            p1 = Point((curr.x+1,curr.y))
            if not (any(x.coords == p1.coords for x in visited)):
                p1.distance=manhattan(p1.coords,goal.coords) #f(n) = g(n) + h(n), but in our case g is constant so we only use h
                stack.append(p1)
            p2 = Point((curr.x,curr.y+1))
            if not (any(x.coords == p2.coords for x in visited)):
                p2.distance=manhattan(p2.coords,goal.coords)
                stack.append(p2)
            p3 = Point((curr.x-1,curr.y))
            if not (any(x.coords == p3.coords for x in visited)):
                p3.distance=manhattan(p3.coords,goal.coords)
                stack.append(p3)
            p4 = Point((curr.x,curr.y-1))
            if not (any(x.coords == p4.coords for x in visited)):
                p4.distance=manhattan(p4.coords,goal.coords)
                stack.append(p4)
            for i in stack:
                if collision(i.coords,obstacles): #if an adjacent point would cause a collision, set its cost to 1001 so we do not visit it
                    i.distance = 10001
                if outofbounds(i.coords,xlim,ylim):
                    i.distance = 1001
            visited.append(curr)
        stack.sort(key=lambda x: x.distance, reverse=False) #sort so that the lowest cost node is visited next
    return visited

    
#set x and y axis of workspace
xaxis = 30
yaxis = 18

#can be set to any qinit and qgoal
qinit = (3,12)
qgoal = (21,2)


#getting coords for obstacles and setting their areas for GUI and collision detection
#use path.Path to create a path connecting the verticies
coord1 = [[4,4], [7,9], [9,7]]
coord1.append(coord1[0]) 
x1, y1 = zip(*coord1) 
fig1area = path.Path([(4,4), (7,9), (9,7)])

coord2 = [[14,3], [14,9], [20,17], [25,11], [20,10], [28,4], [18,7]]
coord2.append(coord2[0])
x2, y2 = zip(*coord2)
fig2area = path.Path([(14,3), (14,9), (20,17), (25,11), (20,10), (28,4), (18,7)])

start1 = Point(qinit)
end1 = Point(qgoal)

astarpath = astar(start1,end1,[fig1area,fig2area],xaxis,yaxis)

#for plotting
pltin = []
for i in astarpath:
    pltin.append(i.coords)
plt.figure()
plt.xlim(0, xaxis)
plt.ylim(0, yaxis)
plt.plot(x1,y1) #drawing figure 1, triangle
plt.plot(x2,y2) #drawing fig 2, polygon

plt.plot(qinit[0],qinit[1],'ro') #plot start
plt.plot(qgoal[0],qgoal[1],'bo') #plot goal
plt.scatter(*zip(*pltin))

plt.show()