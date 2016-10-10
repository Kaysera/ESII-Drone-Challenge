clc; clear

% fuerza de gravedad
g = 9.8;

% masa e inercia del cuadricoptero
m  = 1.840;        % kg
Iz = 0.049066667;  % kg*m^2

% velocidad maxima que alcanzan los motores tipicos
omega_max_rpm = 15000;
omega_max     = omega_max_rpm * 2 * pi / 60;

%empuje máximo que puede generar el motor
T_max  = 1;         % kg
FT_max = T_max * g; % Newtons

% Constante de fuerza que tiene el rotor teniendo en cuenta que
% FT = kFT * w^2
kFT = FT_max / omega_max^2;

%velocidad de sustentacion con 4 rotores
omega_hov4 = sqrt(m*g/4/kFT); 

%velocidad de sustentacion con 2 rotores
omega_hov2 = sqrt(m*g/2/kFT); 

% vamos a calcular kMD a partir de asignar la aceleracion máxima 
% sobre el eje vertical debida al arrastre de los rotores

% supondremos que el drone está inicialmente en reposo y despues 
% de un segundo gira a dos vueltas por segundo
yaw_dot_dot_max_rpm = 120;
yaw_dot_dot_max     = yaw_dot_dot_max_rpm * 2 * pi / 60;

MDR_max = Iz * yaw_dot_dot_max;

kMDR = MDR_max / 2 / omega_hov2^2; 

% Fuerza de arrastre aerodinamico horizontal
Vh_max = 20 * 1000 / 3600;  % velocidad horizontal maxima
roll_max = 30 * 2*pi /360;  % inclinacion maxima

FTh_max = 4 * FT_max * sin(roll_max);
FDh_max = FTh_max;
kFDxy = FDh_max / Vh_max^2;

% Fuerza de arrastre aerodinamico vertical
Vz_max = 3;  % velocidad vertical maxima

FDz_max = FT_max * 4 - m*g;
kFDz = FDz_max / Vz_max^2;

% velocidad limite de descenso
Vz_lim = sqrt(m*g / kFDz);

% momento de arrastre aerodinamico sobre los ejes horizontales
% asumimos un escenario de drone sin gravedad, en el que dos propulsores 
% del mismo lado a maxima potencia pueden hacerlo girar a 2 rev/s 
Vrp_max = 2 * 2*pi;
kMDxy =  2 * FT_max * sin(deg2rad(45))^2 / Vrp_max^2;

% momento de arrastre aerodinamico sobre el eje vertical
Vyaw_max = 2 * 2*pi;
kMDz =  kMDR * (2*omega_hov2^2) / Vyaw_max^2;



