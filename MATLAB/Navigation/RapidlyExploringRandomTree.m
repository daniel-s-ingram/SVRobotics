%P.I. Corke, “Robotics, Vision & Control”, Springer 2017, ISBN 978-3-319-54413-7.  
car = Bicycle('steermax', 0.5);

rrt = RRT(car, 'npoints', 1000);
rrt.plan();
rrt.plot();