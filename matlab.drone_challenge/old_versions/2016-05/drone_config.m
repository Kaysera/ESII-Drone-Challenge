%%
%rosshutdown
clear
clc
rosip = '192.168.58.128';        % casa vmware server ROS Jade  / Gazebo 6.4
%rosip = '161.67.8.48';            % workstation server
rosinit(rosip);      
pause(1);

%% Servosistema con realimentación controlada de estado
%  Se observa que la salida no consigue seguir correctamente a la referencia

load('linsys_x08y04')
A = linsys_x08y04.a;
B = linsys_x08y04.b;
C = linsys_x08y04.c;

disp('Autovalores de A:');
disp(eig(A));
 
disp('Dimensión de A:');
disp(length(A));
disp('Rango de matriz de controlabilidad:');
disp(rank(ctrb(A,B)));


P = [-1 -1 -1 -1 -1.5 -1.5 -2 -2];
K = place(A,B,P);


%% Servosistema con realimentación controlada de estado y salida
%  Hemos dejado como salida solamente la velocidad lineal en el eje del cuerpo
%  Se observa que ...


AA = [ A  zeros(8,4) ;
       C  zeros(4,4) ];
   
BB = [ B          ;
       zeros(4,4) ];
   
disp('Dimensión de AA:');
disp(length(AA));
disp('Rango de matriz de controlabilidad:');
disp(rank(ctrb(AA,BB)));

PP = [-2 -2 -3 -3 -4 -4 -4 -4 -5 -5 -5 -5];

KK = place(AA,BB,PP);
Kx = KK(:,1:8);
Ky = KK(:,9:12)

