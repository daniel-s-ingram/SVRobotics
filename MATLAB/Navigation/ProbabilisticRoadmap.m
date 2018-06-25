%P.I. Corke, “Robotics, Vision & Control”, Springer 2017, ISBN 978-3-319-54413-7.  
load house;

prm = PRM(house);
prm.plan('npoints', 150);
prm.query(place.br3, place.kitchen);
prm.plot();