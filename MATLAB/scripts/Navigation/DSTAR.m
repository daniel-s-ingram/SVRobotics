%P.I. Corke, “Robotics, Vision & Control”, Springer 2017, ISBN 978-3-319-54413-7.  
load house;

ds = Dstar(house);
ds.plan(place.kitchen);
ds.query(place.br3, 'animate');
ds.query(place.garden, 'animate');
ds.query(place.br1, 'animate');