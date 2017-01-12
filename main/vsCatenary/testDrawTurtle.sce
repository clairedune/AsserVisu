// test catenary projection in an image
// when the attached points are mobile
clear;
//close;
exec('../../Load.sce');


s = 0.6;
pose_wMr           = [0,0,0,0,0,30*%pi/180];      // pose in general frame
wMr      = homogeneousMatrixFromPos(pose_wMr);
figure(1)
drawWheeledTurtleTop(wMr,s);
