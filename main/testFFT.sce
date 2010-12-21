clear

sample_freq=1000;
t=0:1/sample_freq:0.5;
s= cos(2*%pi*50*t);

xset("window",1)
plot(t,s)

tf=fft(s);
xset("window",2)
plot(abs(tf(1:$)));

[m,n]=max(abs(tf(1:$/2)))


