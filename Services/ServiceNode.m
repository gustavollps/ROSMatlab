ip = '127.0.0.1';
node_service = robotics.ros.Node('node_service', ip);

%cria server do serviço (node, nome_do_serviço, tipo, callBack)
service_server = robotics.ros.ServiceServer(node_service,'/le_service','roscpp_tutorials/TwoInts',@serviceCallBack)

%cria cliente do serviço (node, serviço)
service_client = robotics.ros.ServiceClient(node_service,'/le_service')

%cria msg do serviço para ser enviado na chamada
request = rosmessage(service_client)

%carrega msg de requisição
request.A = 1
request.B = 2

%chama o serviço - bloqueia código até receber a resposta
response = call(service_client, request)