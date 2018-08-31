masterHost = 'localhost'; %127.0.0.1
node_1 = robotics.ros.Node('node_service', masterHost);

%cria server do serviço (node, nome, tipo, callBack)
server_test = robotics.ros.ServiceServer(node_1,'/le_service','roscpp_tutorials/TwoInts',@serviceCallBack)

%cria cliente do serviço
server_client = robotics.ros.ServiceClient(node_1,'/le_service')

%chama o serviço
response = call(server_client, request)