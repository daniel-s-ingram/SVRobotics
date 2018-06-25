%P.I. Corke, “Robotics, Vision & Control”, Springer 2017, ISBN 978-3-319-54413-7.  
map = LandmarkMap(20);

veh = Bicycle();
veh.add_driver(RandomPath(map.dim));

W = diag([0.1, 5*pi/180].^2);
sensor = RangeBearingSensor(veh, map, 'covar', W, 'angle', [-pi, pi], ...
    'range', 40, 'animate');

ekf = EKF(veh, [], [], sensor, W, []);
ekf.run(1000);

%map.plot();
ekf.plot_map('g');
veh.plot_xy('b');