
pinkBall = Ball('pink');
blackBall = Ball('black');
greenBall = Ball('green');
yellowBall = Ball('yellow');

%Sample Positions that were read by the camera
pos1 = [-31.3916 -37.2592];
pos2 = [16.9905 15.6568];
pos3 = [155.4818 41.2090];
pos4 = [208.2884 -38.3645];

pinkBall.setCentroidPos(pos2);
greenBall.setCentroidPos(pos4);
blackBall.setCentroidPos(pos1);
yellowBall.setCentroidPos(pos3);

blackBall.calcActualPos();
pinkBall.calcActualPos();
yellowBall.calcActualPos();
greenBall.calcActualPos();

pinkBall
blackBall
greenBall
yellowBall
