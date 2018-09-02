ip = '127.0.0.1';

%cria node (nome, ip_do_master)
node_pub = robotics.ros.Node('node_publisher', ip);

%cria publisher (nome, topic, tipo_de_msg)
basicPub = robotics.ros.Publisher(node_pub,'/data','std_msgs/Float32');

%cria mensagem (objeto_publisher)
basicMsg = rosmessage(basicPub);

%carrega msg
basicMsg.Data = 1.54123;

%publica msg
send(basicPub, basicMsg);
