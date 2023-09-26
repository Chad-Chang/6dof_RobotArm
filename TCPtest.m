clear
clc
commu = Communication('127.0.1.1',2011,'/dev/ttyUSB0');
[t,s]=commu.Open(1,0);


while(1)
    commu.send_t(t,"/f/");
end