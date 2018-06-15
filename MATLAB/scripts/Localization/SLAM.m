%P.I. Corke, “Robotics, Vision & Control”, Springer 2017, ISBN 978-3-319-54413-7.  
V = diag([0.02, 0.5*pi/180].^2);
P0 = diag([0.01, 0.01, 0.005].^2);
W = diag([0.1, 1*pi/180].^2);

map = LandmarkMap(20);

veh = Bicycle('covar', V);
veh.add_driver(RandomPath(map.dim));

sensor = RangeBearingSensor(veh, map, 'covar', W, 'angle', [-pi, pi], ...
    'range', 40, 'animate');

ekf = EKF(veh, V, P0, sensor, W, []);
ekf.run(1000);

ekf.plot_map('g');
ekf.plot_xy('r');
veh.plot_xy('b');