clear;

ip = '127.0.0.1';

%cria node (nome, ip_do_master)
node_1 = robotics.ros.Node('node_1', ip);

%registra o publisher
twistPub = robotics.ros.Publisher(node_1,'/pose','geometry_msgs/Twist');
pause(1);

%gera objeto de mensagem para o tópico especificado
twistPubmsg = rosmessage(twistPub);
%carrega mensagem
twistPubmsg.Linear.X = 2;
twistPubmsg.Linear.Y = 1;
twistPubmsg.Linear.Z = 0;

%registra o subscriber
twistSub = robotics.ros.Subscriber(node_1,'/pose');
pause(1);   %tempo para garantir subscribe no tópico
send(twistPub, twistPubmsg);
pause(0.1);   %tempo para callback ser chamada e a mensagem recebida
data = twistSub.LatestMessage %ou    data = receive(twistSub)