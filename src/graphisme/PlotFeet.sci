
function plotFeet(FeetDown,RealRightFootEdges,RealLeftFootEdges)
//
//f=get("current_figure");
// f.figure_size=[1000,400];

 for i = 1:size(FeetDown, 'r')//Needs to be modified for the DS phases
   for j = 1:size(LeftFootEdges,'r')
     if FeetDown(i,4) == 1
       tempx(j) = FeetDown(i,1)+RealLeftFootEdges(j,1)*cos(FeetDown(i,3))-RealLeftFootEdges(j,2)*sin(FeetDown(i,3));
       tempy(j) = FeetDown(i,2)+RealLeftFootEdges(j,1)*sin(FeetDown(i,3))+RealLeftFootEdges(j,2)*cos(FeetDown(i,3));
     elseif FeetDown(i,4) == -1
       tempx(j) = FeetDown(i,1)+RealRightFootEdges(j,1)* cos(FeetDown(i,3))-RealRightFootEdges(j,2)*sin(FeetDown(i,3));
       tempy(j) = FeetDown(i,2)+RealRightFootEdges(j,1)*sin(FeetDown(i,3))+RealRightFootEdges(j,2)*cos(FeetDown(i,3));
     end;
   end;
   xpoly(tempx, tempy);
   pol = gce();
   pol.foreground=color(200-15,200-15,200-15);
end
endfunction


