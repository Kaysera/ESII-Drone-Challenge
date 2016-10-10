
%%
%rosshutdown
clear
clc
rosip = '192.168.203.129';       % casa vmware server ROS Jade  / Gazebo 6.4
%rosip = '192.168.133.128';       % ESII vmware server ROS Hydro / Gazebo 1.9
%rosip = '161.67.8.57';          % workstation server
rosinit(rosip);      
%pause(1);

%%
gazebo = GazeboCommunicator();
% gazebo.pauseSim();
% gazebo.resumeSim();

abejorro = GazeboSpawnedModel('abejorro',gazebo);
%[abejorro_Links,abejorro_Joints] = abejorro.getComponents();


%% Definicion de variables
duration = 1; % Seconds
phys = gazebo.readPhysics();  % Controla la gravedad (on/off)
simActive = true;             % Pausa/reinicia la simulacion

%%

keyObj = KeyInput();
clc
disp('Keyboard Control: ');
disp('Waiting for input: ');

% Use ASCII key representations for reply, initialize here
reply = 0;
while reply ~= 'q'

    reply = getKeystroke(keyObj);
    switch reply
        
        % TRASLACIONES
        case 'w'
            applyForce(abejorro, 'drone', duration, [5 0 0]);
        case 'x'
            applyForce(abejorro, 'drone', duration, [-5 0 0]);
        case 'a'
            applyForce(abejorro, 'drone', duration, [0 5 0]);
        case 'd'
            applyForce(abejorro, 'drone', duration, [0 -5 0]);
        case 'e'
            if phys.Gravity == [0 0 0]
%                applyForce(abejorro, 'drone', duration, [0 0 5]);
                applyForce(abejorro, 'rotor_NW', duration, [0 0 1]);
                applyForce(abejorro, 'rotor_SE', duration, [0 0 1]);
                applyForce(abejorro, 'rotor_NE', duration, [0 0 1]);
                applyForce(abejorro, 'rotor_SW', duration, [0 0 1]);
            else
                applyForce(abejorro, 'drone', duration, [0 0 20]);
            end
        case 'z'
            if phys.Gravity == [0 0 0]
                applyForce(abejorro, 'drone', duration, [0 0 -5]);
%                 applyForce(abejorro, 'rotor_NW', duration, [0 0 -0.1]);
%                 applyForce(abejorro, 'rotor_NE', duration, [0 0 -0.1]);
%                 applyForce(abejorro, 'rotor_SW', duration, [0 0 -0.1]);
%                 applyForce(abejorro, 'rotor_SE', duration, [0 0 -0.1]);
            end
        case 's'
            getState(abejorro);
            setState(abejorro,         ...
                'orientation',[0 0 0], ...
                'linvel'     ,[0 0 0], ...
                'angvel'     ,[0 0 0]);

            
        % ROTACIONES    
        case 'i'     
            applyForce(abejorro, 'drone', 0.1, [0 0 0], [ 0  1  0]);
        case ','     
            applyForce(abejorro, 'drone', 0.1, [0 0 0], [ 0 -1  0]);
        case 'j'     
            applyForce(abejorro, 'drone', 0.1, [0 0 0], [ 1  0  0]);
        case 'l'     
            applyForce(abejorro, 'drone', 0.1, [0 0 0], [-1  0  0]);
        case 'o'     
            applyForce(abejorro, 'drone', 0.1, [0 0 0], [ 0  0  1]);
        case 'm'     
            applyForce(abejorro, 'drone', 0.1, [0 0 0], [ 0  0 -1]);

        % PRUEBAS PUNTUALES
        case ' '     
            disp('    SPACE');
            disp('    -0.1Nm 1s');
            applyForce(abejorro, 'drone', 2, [0 0 0], [0 0 -0.1]);
            pause(0);
            disp('     0.1Nm 1s');
            applyForce(abejorro, 'drone', 1, [0 0 0], [0 0 0.1]);
            
            
            
        % TECLADO NUMERICO
%         case '7'
%             if phys.Gravity == [0 0 0]
%                 applyForce(abejorro, 'rotor_SE', 0.1, [0 0 0.1]);
%             end
%         case '9'
%             if phys.Gravity == [0 0 0]
%                 applyForce(abejorro, 'rotor_SW', 0.1, [0 0 0.1]);
%             end
%         case '1'
%             if phys.Gravity == [0 0 0]
%                 applyForce(abejorro, 'rotor_NE', 0.1, [0 0 0.1]);
%             end
%         case '3'
%             if phys.Gravity == [0 0 0]
%                 applyForce(abejorro, 'rotor_NW', 0.1, [0 0 0.1]);
%             end
            
        % CONTROLES GENERALES    
        case 'g'
            %disp('Keyboard Control: G');
            phys = gazebo.readPhysics();
            if phys.Gravity == [0 0 0]
                phys.Gravity = [0 0 -9.8];
            else
                phys.Gravity = [0 0 0];
            end
            gazebo.setPhysics(phys);
        case 'r'
            %disp('Keyboard Control: R');
            resetWorld(gazebo);
        case 'p'
            if simActive
                pauseSim(gazebo);
            else
                resumeSim(gazebo);
            end
            simActive = ~simActive;
    end
 
end
closeFigure(keyObj);
