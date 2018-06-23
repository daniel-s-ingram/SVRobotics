%P.I. Corke, “Robotics, Vision & Control”, Springer 2017, ISBN 978-3-319-54413-7.
mdl_puma560;

figure(1);
p560.teach;
pause;

figure(2);
T1 = SE3(0.4, 0.2, 0) * SE3.Rx(pi);
T2 = SE3(-0.4, -0.2, 0.2) * SE3.Rx(pi/2);
t = [0:0.05:2]';

q = p560.jtraj(T1, T2, t);
p560.plot3d(q);

figure(3);
qplot(t, q);

pause;
clear all;