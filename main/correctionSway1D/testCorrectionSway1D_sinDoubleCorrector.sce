N=5000;
X=zeros(N,1);
U=zeros(N,1);
B=zeros(N,1);
x=5;
xdot=0;
b=0;
bdot=0;
k=1;
umax=.3;
T=1e-2;
PG=1; // <- switch 0=sans prise en compte du PG, 1=prise en compte du PG
for i=1:N,
 
 u=-k*((x-PG*b)+2*(xdot-PG*bdot));
 if (u<-umax) u=-umax; end;
 if (u>umax) u=umax; end;
 
 
 v=u-(4*(sin(T*i)*T^2*i*N-sin(T*i)*T^2*i^2-2*cos(T*i)*T*N+4*cos(T*i)*T*i+2*sin(T*i)))/T^2/N^2;
 
 if i>N/2 err=.27; else err=0; end;
 
 x=x+T*xdot+T*T/2*(v+err);
 xdot=xdot+T*(v+err);
 
 b=b+T*bdot+T*T/2*(v-u);
 bdot=bdot+T*(v-u);

 B(i)=b;
 X(i)=x;
 U(i)=u;

end;
plot([X,U,X-B]);
