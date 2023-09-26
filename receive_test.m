function a = receive_test(message)
    data_uart = message;
    if(data_uart(1)=='/')
        i = 2;
        while(data_uart(i)~='/')
            i= i+1;
        end
        a = data_uart(2:i-1);
    end

end