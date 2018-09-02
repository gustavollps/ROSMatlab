ip = '127.0.0.1'; %127.0.0.1

%cria node (nome, ip_do_master)
node_sub = robotics.ros.Node('node_subscriber', ip);

%registra subcriber com callBack
basicSub = robotics.ros.Subscriber(node_sub,'/data',@callBack);

