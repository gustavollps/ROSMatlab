publisher = rospublisher('/turtle1/cmd_vel');

msg = rosmessage(publisher);

msg.Linear.X = 2;

while true
    msg.Angular.Z = (rand*2 -1) * 10;
    send(publisher,msg);
    pause(0.1);
end    
