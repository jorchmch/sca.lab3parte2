clc,clear;
ap=[0 1 0 0;24.036 0 0 0;0 0 0 1;-0.239 0 0 0]; 
bp=[0;-4.789;0;1.437];
cp=[1 0 0 0;0 0 1 0 ];
dp=[0;0];

sys = syslin("c",ap,bp,cp,dp);

Q = cp'*cp;
R=1*eye(2,2);
//R=eye(4,4);
[H,kkkk] = lqe(sys,Q,R)   //H ganancia de Observador
clear kkkk;
// controlador
R2=[1]; 
[G,kkkk]=lqr(sys,Q,R2)  // G ganancia del controlador
clear kkkk;

// CONTROLADOR 
A11=ap-bp*G;
A12=-bp*G;
A21=H*cp;
A22=ap-H*cp-bp*G;

Ak=[A11 A12;A21 A22];   // 8x8 Ak
Bk=[zeros(4,2);-H];      // 8x4 Bk
Ck=[cp zeros(2,4)];     // 4x8
Dk=zeros(2,2);
//Bk=[zeros(4,4);-H];      // 8x4 Bk
//Ck=[cp zeros(4,4)];     // 4x8
//Dk=zeros(4,4);
INIT=[0 ;0 ;0 ;0 ;0 ;0 ;0 ;0];
INIT1=[0 ;0 ;0 ;0 ];

sK=syslin("c",Ak,Bk,Ck,Dk,INIT);  // controlador discretizado

dt=0.1;
sysKdis=cls2dls(sK,dt);     // DISCRETO COMPENSADOR..en espacio de Estados
sysGdis=cls2dls(sys,dt);     // DISCRETO PLANTA en espacio de Estados
SLLT=ss2tf(sysKdis);        // en Funcion Transferencia

// conversion
// los coeficientes corresponden
// z^0 z^1 z^2...
num1_1=coeff((SLLT(1,1).num));
den1_1=coeff((SLLT(1,1).den));
num1_2=coeff((SLLT(1,2).num));
den1_2=coeff((SLLT(1,2).den));
num2_1=coeff((SLLT(2,1).num));
den2_1=coeff((SLLT(2,1).den));
num2_2=coeff((SLLT(2,2).num));
den2_2=coeff((SLLT(2,2).den));

// def de x
x=[1 zeros(1,50)]
y11=filter(num1_1,den1_1,x);
y12=filter(num1_2,den1_2,x);
y21=filter(num2_1,den2_1,x);
y22=filter(num2_2,den2_2,x);

// ploteo
T=0:50;
subplot(2,2,1); plot(T,y11); title('Ec.Dif IN1-OUT1');
subplot(2,2,2); plot(T,y12); title('Ec.Dif IN1-OUT2');
subplot(2,2,3); plot(T,y21); title('Ec.Dif IN2-OUT1');
subplot(2,2,4); plot(T,y22); title('Ec.Dif IN2-OUT2');

// SISTEMA EN LAZO CERRADO
LCA11=ap-bp*G;
LCA12=bp*G;
LCA21=zeros(4,4);
LCA22=ap-H*cp;

LCA=[LCA11 LCA12;LCA21 LCA22];   // 8x8 A
LCB=[zeros(4,2);H];      // 8x2 B
LCC=[cp zeros(2,4)];
LCD=zeros(2,2)
//LCB=[zeros(4,4);H];      // 8x2 B
//LCC=[cp zeros(4,4)];
//LCD=zeros(4,4)
LCINIT=[0 ;0 ;0 ;0 ;0 ;0 ;0 ;0];

LC=syslin("c",LCA,LCB,LCC,LCD,LCINIT);
LCdis=cls2dls(LC,dt); 
