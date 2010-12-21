//------------------------------//
//      point2DDrawCarre
//------------------------------//
function point2DDrawCarre(x,cote,col)
 // affiche un carre centre en '(x,y)', de cote 'cote' et de couleur 'col'
 xrect(x(1)-cote/2,x(2)+cote/2,cote,cote) 
 hr = gce ();
 hr.thickness=1;
 hr.foreground=col;
 hr.background=col;
 hr.fill_mode = "on";
endfunction

//------------------------------//
//      point2DDraw
//------------------------------//
function point2DDraw(x,cote,col)
  xfarc(x(1)-cote/2,x(2)+cote/2,cote,cote,0,64*360);
  e=gce()//dernière entité (Arc)
  e.background=col;
endfunction

//------------------------------//
//      mireDraw
//------------------------------//
function mire2DDraw(x,cote,col)
// display a n dots target
nbpoints=length(x)/2;
for i=1:nbpoints
   point2DDraw(x((i-1)*2+1:(i-1)*2+2),cote,col);
end;

endfunction

//-----------------------------//
// mireEvolutionDraw
// date 5/01/2010
// author claire dune
//------------------------------//
function mireEvolutionDraw(Np,M,width)
  // M contains the points in time:
  // p1x(t1) 
  // p1y(t1) 
  //   |
  // p5x(t1) 
  // p5y(t1)
  //   |
  // p1x(tNp) 
  // p1y(tNp)
  //   |
  // p5y(tNp)
  //
  //width is the line width
  //
  // example:
  // M = ga_presHorGlobal3dMire
  // hf2d = createPlanImage(1,"Point 2D");
  // mireEvolutionDraw(M,3);
  // show_pixmap()
  
  // number of points = number of lin2/2 
   Nbpts = length(M)/(Np*2);
   for k=1:Nbpts 
    x=[];
    y=[];
    for i=1:Np
      index1 = Nbpts *2*(i-1)+(k-1)*2+1;
      index2 = Nbpts *2*(i-1)+(k-1)*2+2;
      x = [x,M(index1) ] ;
      y = [y,M(index2) ] ;
    end
      xset("color",k)
      plot(x,y,'rO')
      xpoly(x,y,"lines",0) // by default closed
      // News graphics only
      e=gce(); // get the current entity (the last created: here the polyline)
      //e.closed = 'off' // the polyline is now open
      e.thickness=width;

  end
    
endfunction  

