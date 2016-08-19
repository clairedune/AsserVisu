



figure(1);
a = gca();
a.box = "off";
a.filled = "on";
a.isoview = "on";
a.data_bounds = [-2;2;-2;2];
a.grid=[1,1];


for t=0:0.1:1
    
    
    robot1 = [t;0;0;0;0;0.3];
    robot2 = [t;1;0;0;0;0.2];
    
    delete(a.children);
    drawlater();
    drawTurtleSide(robot1(1),robot1(3),0.6);
    drawTurtleSide(robot2(1),robot2(3),0.6);
    
       
    
    drawnow();
    sleep(100);
end

