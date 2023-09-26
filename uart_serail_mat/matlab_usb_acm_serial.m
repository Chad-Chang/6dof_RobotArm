while true
    s = serialport('/dev/ttyUSB0',115200);
    s2  = serialport('/dev/ttyACM0',115200);
    fopen(s);
    fopen(s2);
    Traj =1 ;
    % c = "["+1.2+","+(-90)+","+(-90)+","+0+","+1234+","+234+","+0+"]";
    c = "["+1+","+(-80)+","+(-80)+","+0+","+23+","+234+","+234+"]";
    t = "{"+2+","+(-3)+","+(-4)+","+5+","+6+","+7+","+8+"}";
    fprintf(s,c);
    fprintf(s2,t);
    fclose(s);
    fclose(s2);
    clear s;
    clear s2;
end

