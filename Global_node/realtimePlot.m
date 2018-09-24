clear;
%cria subscriber
subscribe = rossubscriber('/turtle1/pose',@callback);
pause(2);
%variavel global para armazenar msg do tópico /turtle1/pose (atualizada na
%callBack
global pose;

%fechar figuras anteriores
close all;
%nova figura
fig = figure;

hold on

xlim([0 11.1]); 
ylim([0 11.1]);

while true
    clf(fig);
    scatter(pose.X,pose.Y,'')  %plot ponto com a localização
    xlim([0 11.1]); 
    ylim([0 11.1]);            
    
    %pausa para não jogar um loop INFINITO-INFINITAMENTE rápido
    pause(1/62.5);
end