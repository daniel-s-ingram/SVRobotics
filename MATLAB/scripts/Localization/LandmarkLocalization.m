%P.I. Corke, “Robotics, Vision & Control”, Springer 2017, ISBN 978-3-319-54413-7.  
map = LandmarkMap(20, 10);

V = diag([0.02, 0.5*pi/180].^2);
veh = Bicycle('covar', V);
veh.add_driver(RandomPath(map.dim));

P0 = diag([0.005, 0.005, 0.001].^2);
W = diag([0.1, 1*pi/180].^2);
sensor = RangeBearingSensor(veh, map, 'covar', W, 'angle', [-pi, pi], ...
    'range', 40, 'animate');

ekf = EKF(veh, V, P0, sensor, W, map);
ekf.run(1000);

%map.plot();
veh.plot_xy('b');
ekf.plot_xy('r');
ekf.plot_ellipse('k');