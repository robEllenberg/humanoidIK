function [Gp,Ke,Kx,sys]=previewGains(delt,zc)

g = 9.81;

A = [1 delt delt^2/2;
    0 1 delt;
    0 0 1];
B = [delt^3/6;
    delt^2/2;
    delt];
C = [1 0 -zc/g];

A1 = [1 C*A;zeros(3,1) A];
B1 = [C*B;B];
%C1 = [1 0 0 0];

Qe = 1;
Qx = zeros(3,3);
%Qx = ones(3,3);

Q = [Qe zeros(1,3);zeros(3,1) Qx];
R = 1e-6;

% P = solution of DARE, G = optimal gain
[P,~,G] = dare(A1,B1,Q,R);

% Optimal gain from DARE
Ke = G(1,1);
Kx = G(1,2:end);

%% Preview Controller Design
% Preview size
% N = [x1 x2 x3];
N = 0:delt:1.5;

% Calculation of Preview gain
% recursively computed
Ac = A1-B1*G;
Gp = zeros(1,length(N)); % Preview gain matrix
X1 = zeros(4,length(N)); % Preview state

Gp(1) = -Ke;
X1(:,1) = -Ac'*P*[1;0;0;0];

for i = 2:length(N)
    Gp(i) = (R+ B1'*P*B1)\B1'*X1(:,i-1);
    X1(:,i) = Ac'*X1(:,i-1);
end
sys=ss(A,B,C,0);
end