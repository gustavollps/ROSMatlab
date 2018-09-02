function callBack(~,msg)
    %cria variável global para ser usada fora
    global data;
    
    %recebe informação na variavel global
    data = msg.Data;
    
    %debug pra saber que a callBack foi cahamada (só para teste)
    disp('Chamou a callBack')
end