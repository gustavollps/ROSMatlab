clear;
MODO_LGBT = false;
%cria subscriber
subscribe = rossubscriber('/turtle1/pose',@callback);

%variavel global para armazenar msg do tópico /turtle1/pose (atualizada na
%callBack
global pose;

%fechar figuras anteriores
close all;
%nova figura
fig = figure;

if MODO_LGBT == true
    xlim([0 11.1]); 
    ylim([0 11.1]);
end

hold on

while true
    if MODO_LGBT == false
        clf(fig);   %limpa figura  
    end
    scatter(pose.X,pose.Y,'')  %plot ponto com a localização
    
    if MODO_LGBT == false
        %define limites dos eixos X e Y
        xlim([0 11.1]); 
        ylim([0 11.1]);
    end
    
    %pausa para não jogar um loop INFINITO-INFINITAMENTE rápido
    pause(1/62.5);
end