%P.I. Corke, “Robotics, Vision & Control”, Springer 2017, ISBN 978-3-319-54413-7.  
load house;

bug = Bug2(house);
bug.plot();
bug.query(place.br3, place.kitchen, 'animate');