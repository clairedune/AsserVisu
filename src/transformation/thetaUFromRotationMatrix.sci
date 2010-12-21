function [r,theta] = thetaUFromRotationMatrix(R)
// auteur CLaire dune
// Date novembre 2009
// Mouvement de rotation

s = (R(2,1)-R(1,2))*(R(2,1)-R(1,2))+(R(3,1)-R(1,3))*(R(3,1)-R(1,3))+ (R(3,2)-R(2,3))*(R(3,2)-R(2,3));
s = sqrt(s)/2.0;
c = (R(1,1)+R(2,2)+R(3,3)-1.0)/2.0;
theta=atan(s,c);  



if (s > %eps) | (c > 0.0) 
      if(theta<%eps)
        sinca = 1 ; 
      else
        sinca = s/theta;
      end;
      r(1) = (R(3,2)-R(2,3))/(2*sinca);
      r(2) = (R(1,3)-R(3,1))/(2*sinca);
      r(3) = (R(2,1)-R(1,2))/(2*sinca);
     
else 
      r(1) = theta*(sqrt((R(1,1)-c)/(1-c)));
      if ((R(3,2)-R(2,3)) < 0) 
        r(1) = -r(1);
      end
      r(2) = theta*(sqrt((R(2,2)-c)/(1-c)));
      if ((R(1,3)-R(3,1)) < 0) 
        r(2) = -r(2);
      end
      r(3) = theta*(sqrt((R(3,3)-c)/(1-c)));
      if ((R(2,1)-R(1,2)) < 0) 
        r(3) = -r(3);
      end
end
 


endfunction
