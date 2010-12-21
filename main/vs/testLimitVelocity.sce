// test limit velocity
clear 
getd("src/asserVisu")

v0 = [0.2 -0.3 .4];

t  = 0 : 0.01 : 1;

v  = [v0(1)*exp(-t)'   v0(2)+0.8*t' v0(3)-0.4*t'];

plot(v)

vcamSat =[];

for iter=1:length(t)

  vcamSat = [vcamSat ; satVelo(v(iter,:),iter,0.02,[0.2 0.2 0.2],0,%F)];
    
  
  
  
  
end


plot(vcamSat,'-.');