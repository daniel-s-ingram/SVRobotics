%P.I. Corke, “Robotics, Vision & Control”, Springer 2017, ISBN 978-3-319-54413-7.  
V = diag([0.02, 0.5*pi/180].^2);

veh = Bicycle('covar', V);
veh.add_driver(RandomPath(10));
veh.run();
veh.Fx([0, 0, 0], [0.5, 0.1]);

P0 = diag([0.005, 0.005, 0.001].^2);
ekf = EKF(veh, V, P0);
ekf.run(1000);

veh.plot_xy('b');
hold on;
ekf.plot_xy('r');