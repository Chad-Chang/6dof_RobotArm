while true
    s2  = serialport('/dev/ttyACM0',115200);
    fopen(s2);
    t = "{"+1+","+(-80)+","+(-80)+","+0+","+23+","+234+","+234+"}";
    fprintf(s2,t);
    fclose(s2);
    clear s2;
end