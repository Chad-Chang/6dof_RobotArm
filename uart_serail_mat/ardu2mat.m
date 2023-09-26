serialportlist("available")
s = serialport('/dev/ttyUSB0',115200);
fopen(s);
a = 1;
flush(s);
while s.NumBytesAvailable == 0 end

while s.NumBytesAvailable
    data = char(readline(s))
    data = data(1:length(data)-1)
    length(data)
    class(data)
    if(data == '/c/')
        disp("c")
    end
    if(data == '/z/')
        disp("end")
        a = 0;
        clear s
        break
    end
end