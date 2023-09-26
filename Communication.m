classdef Communication % server communication
    properties
        m_ip = [];
        m_tcp_port = [];
        m_Ser_port = [];
    end
%% methods
    methods
%% define
        function obj = Communication(ip,tcp_port,serial_port)  % ,2016,/dev/ttyUSB0
            obj.m_ip = ip;
            obj.m_tcp_port = tcp_port;
            obj.m_Ser_port = serial_port;
        end
%% open
        function [t,s] = Open(obj,tcp,serial)
            if tcp ==1
                t = tcpserver(obj.m_ip, obj.m_tcp_port, "Timeout",20);
                set(t, 'InputBufferSize', 1000);
                fopen(t)
            else
                t= 0;
            end
            if serial==1
                s = serialport(obj.m_Ser_port,115200);
                fopen(s);
            else
                s = 0;
            end


            
        end
%% sending_tcp        
        function send_t(obj,t,message_tcp) % can check status if add print trajecotry
            fprintf(t,message_tcp);
%             disp("sent tcp");
            message_tcp
        end
%% receive_tcp
        function data_tcp = receive_t(obj,t)
            while t.BytesAvailable == 0 end
            while t.BytesAvailable ~=0
                data_tcp = read(t,t.BytesAvailable,'string')
            end
        end
%% homogeneous TF transmission
%         function data = receive_t_TF(obj,t)
%             while t.BytesAvailable == 0 end
%             while t.BytesAvailable ~=0
%                 data = fread(t,t.BytesAvailable,'string');
%             end
%         end
%% send_serial_angle
        function send_s(obj,s,status,q) % can check status if add print trajecotry
            c = "["+'a'+","+status+","+rad2deg(q(1))+","+rad2deg(q(2))+","+rad2deg(q(3))+","+rad2deg(q(4))+","+rad2deg(q(5))+","+rad2deg(q(6))+"]";
            fprintf(s,'%s\n',c);
%             disp("sent_angle")
        end
%% send_serial_depth
        function send_s_depth(obj,s,depth) % can check status if add print trajecotry
            c = "[" +'b' + "," + depth + "]"
            fprintf(s,'%s\n',c);
        end
%% receive_serial
        function [ret] = receive_s(obj,s) %will receive single word
            while s.NumBytesAvailable == 0 end
            while s.NumBytesAvailable
                data_uart = char(readline(s));
                data_uart = data_uart(1:length(data_uart)-1);
                if(data_uart(1)=='/')
                    i = 2;
                    while(data_uart(i)~='/')
                        i= i+1;
                    end
                    ret = data_uart(2:i-1);
                else
                    ret = false;
                end
%                 if(data_uart(1) == '/')
%                     disp("pass1")
%                     if(data_uart(3) == '/')
%                         disp("pass2")
%                         ret = data_uart(2);
%                     else
%                         ret = false;
%                     end
%                 else
%                     ret = false;
%                 end
            end
        end

%% ending
        function Close(obj,t,s) %choose what to delete
            if isempty(t) ~= 1
                clear t
            end
            if isempty(s) ~= 1
                clear s
            end
        end

    end

end

