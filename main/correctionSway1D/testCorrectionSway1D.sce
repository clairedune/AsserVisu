// 1d servo of a position x to zero
// when the PG disturbs the control with an additional sin
// comparison of 2 methods
// 1) cancelling the effect of the sin in the control
// 2) compensate for the sin

clear


// --- Variables of 1st method 
x  = 5;   // initial position
c  = 0;   // initial correction
u  = 0;   // initial control 
v  = 0;   // initial control+sin
X  = [0]; // position trajectory
U  = [0]; // control trajectory
V  = [0]; // control+sin trajectory
C  = [0]; // "correction"

// --- Varibles of 2nd method
xp = 5;   // idem method 1 
up = 0;  
vp = 0;
XP = [0];
UP = [0];
VP = [0];

// --- Control variable
k  = 0.6; // gain >0
T  = 1e-1;// sampling period
N  = 400; // number of iterations


// --- Begin control Loop
for i=1:N,
 // compute the additional sin 
 b=3*sin(T*i);
 
 // --- 1st Method
 // estimate the mean of the current correction
 if(i<=floor(2*%pi/T))
    E=mean(C);
 else
    E=mean(C(i-floor(2*%pi/T):i-1));
 end
 // E=0;
 //c=0;
 
 //cancelling the sway motion
 u=-k*(x-(c-E));
 if(abs(u)>1)
  u=sign(u)*1;
 end
 v=u+b;
 x=x+T*v;
 c=c+T*(v-u);
 
 // --- 2nd Method
 //suppressing the preview sway
 up=-k*(xp)-b;
 if(abs(up)>1)
  up=sign(up)*1;
 end
 vp=up+b;
 xp=xp+T*vp;
 
 // store data for display 
 C=[C; c];
 X=[X; x];
 U=[U; u];
 V=[V; v];
 XP=[XP; xp];
 UP=[UP; up];
 VP=[VP; vp];

end;// --- end of the loop

// --- Display
xset("window",1);
plot(X,'g')
plot(U,'b')
plot(V,'r')

xset("window",2);
plot(XP,'g')
plot(UP,'b')
plot(VP,'r')



