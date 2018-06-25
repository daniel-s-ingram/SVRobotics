%P.I. Corke, “Robotics, Vision & Control”, Springer 2017, ISBN 978-3-319-54413-7.  
lp = Lattice();

for i=1:10
    lp.plan('iterations', i);
    lp.plot();
    drawnow;
    pause;
end

lp.query([0 0 0], [7 6 pi]);
lp.plot();