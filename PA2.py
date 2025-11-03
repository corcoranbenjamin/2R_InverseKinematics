import numpy as np #import numpy for numerical operations
import matplotlib.pyplot as plt #import matplotlib for plotting
from matplotlib.patches import Circle #import circle patch for plotting
from scipy.interpolate import splprep, splev #import splprep and splev for spline interpolation

#initialize link lengths
L1 = 1.0
L2 = 0.5

#define workspace annulus
minWS = abs(L1 - L2)  
maxWS = L1 + L2     

#create a list to store user selected points
points = []

#function to get selected points
def GetPoints(event):
    
    #check if the click is on the figure
    if event.inaxes == figure:

        #get the x and y coordinates of the point
        x = event.xdata;
        y = event.ydata; 

        #calculate r, the distance from the origin to the point
        r = np.sqrt(x*x + y*y)
        
        #check if point is within the workspace, if so add it to the list and plot it
        if minWS <= r <= maxWS:
            
            #append the point to the list and plot it
            points.append((x, y))
            
            #plot black circle dot at the click
            figure.plot(x, y, 'ko', markersize=1)
            plt.draw()

            print(f"Point {len(points)}: ({x}, {y})")


#function to smooth the path after at least 5 points are selected and user presses enter
def SmoothPath(event):
    
    #if user presses enter and at least 5 points are selected, smooth the path
    if event.key == 'enter':
        if len(points) >= 5:
            
            #convert points to array for spline interpolation
            pointsArray = np.array(points)
            x = pointsArray[:, 0]
            y = pointsArray[:, 1]
            
            
            #s is set to 0.00 so that path precisely passes through all points
            #tck --> tuple containing the spline coefficients and the order of the spline
            #u --> parameter values for the spline
            tck, u = splprep([x, y], s=0.00, per=False)
            
            #get smoothed x and y values in 2x200 array
            pointsOnSpline = np.linspace(0, 1, 200)
            smoothedX, smoothedY = splev(pointsOnSpline, tck)
            smoothedPath = np.column_stack((smoothedX, smoothedY))
            
            #plot the smooth path 
            figure.plot(smoothedX, smoothedY, 'k-')
        
            #update the plot
            plt.draw()

            #call animate to start the animation 
            animate(smoothedPath)


#function returns theta 1 and 2 for a given x, y position
def InverseKinematics(x, y): 
    
    #calculate theta 1, 2 using the inverse kinematics equations
    theta2 = np.acos((x**2 + y**2 - L1**2 - L2**2) / (2*L1*L2))
    theta1 = np.arctan2(y, x) - np.arctan2(L2*np.sin(theta2), L1 + L2*np.cos(theta2))

    return theta1, theta2; 


#function calulates the joint locations given theta1 and theta2
def ForwardKinematics(theta1, theta2):
 
    #calculate joint positions for animation using forward kinematics
    x1 =  L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)
    x2 = L2 * np.cos(theta1 + theta2) + x1
    y2 = L2 * np.sin(theta1 + theta2) + y1

    return x1, y1, x2, y2


#function animates the robot following the smooth path 
def animate(smoothPath):

    figure.set_title('Animating path...')

    #create two line objects for link 1 and 2
    link1, = figure.plot([], [], 'o-', linewidth=3, color='orange')
    link2, = figure.plot([], [], 'o-', linewidth=3, color='orange')
    
    #iterate over each (x,y) point in the smooth path array
    for i in range(len(smoothPath)): 
    
        #get theta 1, 2 using inverseKinematics for current x, y position 
        #i is the index of the current point in the smoothPath array 
        x = smoothPath[i, 0]
        y = smoothPath[i, 1]
        theta1, theta2 = InverseKinematics(x, y);

        #calculate link coordinates using forward kinematics
        x1, y1, x2, y2 = ForwardKinematics(theta1, theta2);

        #update link positions for current frame
        link1.set_data([0, x1], [0, y1])
        link2.set_data([x1, x2], [y1, y2])

        plt.draw()
        plt.pause(0.05)

#create figure object for plotting
fig, figure = plt.subplots(figsize=(7, 7))

#add minimum and maximum workspace circles to show annulus (area between inner and outer circles)
innerCircle = Circle((0, 0), minWS, fill=False, edgecolor='red', linewidth=1)
outerCircle = Circle((0, 0), maxWS, fill=False, edgecolor='blue', linewidth=1)

figure.add_patch(innerCircle)
figure.add_patch(outerCircle)
figure.set_xlim(-maxWS - 0.5, maxWS + 0.5)
figure.set_ylim(-maxWS - 0.5, maxWS + 0.5)
figure.set_xlabel('X (meters)')
figure.set_ylabel('Y (meters)')

figure.set_title('Click on 5 or more points then press Enter')

#connect event handles to the figure for getting points, smoothing path 
fig.canvas.mpl_connect('button_press_event', GetPoints)
fig.canvas.mpl_connect('key_press_event', SmoothPath)

plt.show()
