clear;

ip = '127.0.0.1'; %ip
node_timer = robotics.ros.Node('node_timer', ip);

timerPub = robotics.ros.Publisher(node_timer,'/timer_topic','std_msgs/String')
timerMsg = rosmessage(timerPub);


%cria struct como parametro para passar para o timer, no caso o publisher o
% e a msg
param.pub = timerPub;

%variavel global para ser usada dentro do timer
global i
i=0

%cria timer (time_delay, {callBack, param1, param2, ...})
rosTimer = ExampleHelperROSTimer(1, {@timerCallBack,param});