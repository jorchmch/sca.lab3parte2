% % https://octave-online.net/bucket~KfDr1kERxjkZDP7LrDMhWL
clc,clear;
ap=[0 1 0 0;24.036 0 0 0;0 0 0 1;-0.239 0 0 0]; 
bp=[0;-4.789;0;1.437];
cp=[1 0 0 0;0 0 1 0 ];
dp=[0;0];

% Controllability 
%
cm = ctrb(ap,bp)
disp('rango de controlabilidad')
rcm= rank(cm)
% Observability
%
om = obsv(ap,cp)           
disp('rango de observabilidad')
rom = rank(om)


% para filtro kalman
sys=ss(ap,bp,cp,dp);
Q =cp'*cp;
R=[0.01 0 ; 0 0.01];
[H, ~, ~] = lqe(sys,Q,R)
% controlador
R2=[0.01]; 
[G, ~, ~]=lqr(ap,bp,Q,R2)  % GANANCIA DEL CONTROLADOR


% dinamica en lazo cerrado
A11=ap-bp*G;
A12=bp*G;
A21=zeros(4,4);
A22=ap-H*cp;

Ak=[A11 A12;A21 A22];   % 8x8 Ak
Bk=[zeros(4,2);H];      % 8x2 Bk
Ck=[cp zeros(2,4)];
Dk=[0 0;0 0];

sysK=ss(Ak,Bk,Ck,Dk);


%% respuesta al impulso
[y11,t11] = step(sysK,[0:0.01:10]);

figure
disp('RESPUESTA AL ESCALON UNITARIO')
plot(t11,y11(:,1,1));grid;
hold
plot(t11,y11(:,2,2));grid;
title('Funcion Transferencia (1,1) y (2,2)')
xlabel('Time (s)')
ylabel('Amplitude')
legend('tf11','tf22')

figure
plot(t11,y11(:,1,2));grid;
hold
plot(t11,y11(:,2,1));grid;
xlabel('Time (s)')
ylabel('Amplitude')
title('Funcion Transferencia (1,2) y (2,1)')
legend('tf12','tf21')
hold off;


%% respuesta a impulso inicial
figure
t=0:0.01:10; 
x=initial(sysK,[1;0;2;0;0;0;0;0],t);
disp('RESPUESTA AL IMPULSO')
plot(t,x)
xlabel('Time (s)')
ylabel('Amplitude')
title('Respuesta la condicion inicial ')
grid on;
