function Ai=matrizAi(theta,di,ai,alfa)
Ai = [cos(theta) -sin(theta)*cos(alfa) sin(theta)*sin(alfa) ai*cos(theta);
      sin(theta) cos(theta)*cos(alfa) -cos(theta)*sin(alfa) ai*sin(theta);
      0 sin(alfa) cos(alfa) di;
      0 0 0 1];
end