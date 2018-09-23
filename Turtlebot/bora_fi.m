clear;

ip = '127.0.0.1' %NÃO COPIA ESSA PORRA
node_vel = robotics.ros.Node('vai_fi',ip);

pub = robotics.ros.Publisher(node_vel,...
    '/mobile_base/commands/velocity',...
    'geometry_msgs/Twist');

msg = rosmessage(pub);

msg.Linear.X = -0.5;
msg.Angular.Z = 0.0;

while true
   send(pub,msg);
   pause(0.1);
end