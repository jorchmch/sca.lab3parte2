% % https://octave-online.net/bucket~PhxdoTFwcvmrEwH7pgHGEt
clc,clear;
ap=[0 1 0 0;24.036 0 0 0;0 0 0 1;-0.239 0 0 0]; 
bp=[0;-4.789;0;1.437];
cp=[1 0 0 0;0 0 1 0 ];
dp=[0;0];
% para filtro kalman
sys=ss(ap,bp,cp,dp);
Q =cp'*cp;
R=[0.01 0 ; 0 0.01];
[H, ~, ~] = lqe(sys,Q,R)
% controlador
R2=[0.01]; 
[G, ~, ~]=lqr(ap,bp,Q,R2)  % GANANCIA DEL CONTROLADOR

%cONTROLADOR k(S)

K11=[ap-bp*G];  % 4X4
K12=[-bp*G];    %4X4
K21=[H*cp]; %4x4
K22=[ap-H*cp-bp*G];   %2x2

AK=[K11 K12;K21 K22];   %8x8
BK=[zeros(4,2);-H];     %8x2
CK=[cp zeros(2,4)];     %2x8
DK=[0 0;0 0];
% % % % % % % % % % % % % % % 
sysK=ss(AK,BK,CK,DK)


pause;
%% respuesta al impulso
[y11,t11] = step(sysK,[0:0.01:10]);

figure
plot(t11,y11(:,1,1));grid;
hold
plot(t11,y11(:,1,2));grid;
plot(t11,y11(:,2,1));grid;
plot(t11,y11(:,2,2));grid;
xlabel('Time (s)')
ylabel('Amplitude')
title('output 1 response caused by input  ')
hold off;

disp('GRAFICA IMPULSO INICIAL DEL ESPACIO DE ESTADOS DEL COMPENSADOR')
%% respuesta a impulso inicial
figure
t=0:0.01:10; 
x=initial(sysK,[1;0;0;0;0;0;0;0],t);
plot(t,x)
grid on;



disp('GRAFICAS ESCALON DEL ESPACIO DE ESTADOS DEL COMPENSADOR')
%% respuesta al impulso
[yk1,tk1] = step(sysK,[0:0.01:10]);

figure
plot(tk1,yk1(:,1));grid;
hold
plot(tk1,yk1(:,2));grid;
xlabel('Time (s)')
ylabel('Amplitude')
title('output 1 response caused by input  ')
hold off;

pause;

KS=tf(sysK);
GN=tf(sys);
P=KS*GN;    %1x2
UNO=ones(2,1);
paso1=UNO+KS*GN;

LC1=1/paso1(1,1);
LC2=1/paso1(2,1);

figure
step(LC1,0:0.01:5)
hold on
step(LC2,0:0.01:5)
