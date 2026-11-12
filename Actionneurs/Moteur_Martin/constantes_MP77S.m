%% Constantes de la MCC MP77S
Vmax=48;

Imax=12.2;

k=0.14;                 % constante de fem : Nm/A=V/(rad/s)

R=0.9;                 % résistance série : ohms (0.25 dans la doc technique)

L=0.5e-3;               % inductance série : H

J=0.002;    % inertie : kg.m^2 (16/10000 dans la doc technique)

f=0.037*60/(1000*2*pi); % constante de frottements fluides : N.m/(rad/s)

Gamma0=0.2;             % frottements secs : N.m (0.1 dans la doc technique)

Tee=L/R;                % constante de temps électrique

Tm=J/f;                 % constante de temps mécanique
