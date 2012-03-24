function angles=stepIKSolve(unscaledPosition,q0) %#codegen
%% IK Solver (batch)
% This takes a trajectory of goal positions and outputs the corresponding
% configuration angles.  The IKSolver variable contains the
% problem-specific function references, such as the Jacobian, forward
% kinematics, etc.
%
%
% Tips if something fails:
% 1)An arm or leg may be trying to extend past a singularity.
% 2) There may be a singularity that coincides with some pose...try playing
% with the parameters a bit to see if it goes away.
% 3) The solution tolerance is too tight.  Look at the "tol" variable in
% the IKSolve function

%h=waitbar(0,'1','Name','Solving IK...','CreateCancelBtn','delete(gcbf)');
%positions=unscaledPositions-repmat(p0,1,size(unscaledPositions,2));
position=unscaledPosition;

tol=.0005;

angles=solutionStep(position,q0,[]);

tempangles=angles;
j=0;
completedSolution=0;
while ~completedSolution
    tempangles=solutionStep(position,tempangles,[]);
    temppositions=miniForwardKinematics(tempangles);
    j=j+1; 
    positionError=norm(temppositions-position);

    if positionError<tol
        angles=tempangles;
        completedSolution=1;
    elseif j>30
        break;
    end   
end

if ~completedSolution
    %Assign original position if tolerances are too great
	angles=q0;
end
completedSolution;
end

function dq=clampMaxAbs(Psi,gamma)
[~,n]=size(Psi);
Gamma=[repmat(gamma,n,1),zeros(n,n-length(gamma))];
%fugly way to do ..
qtemp=(abs(Psi)<=Gamma).*Psi+(abs(Psi)>Gamma).*Gamma.*sign(Psi);
dq=sum(qtemp,2);
end

function q=solutionStep(position,q0,~)
% Single iteration of least squares solution
J=miniJacobian(q0);
r0=miniForwardKinematics(q0);
% Change here for different methods (crude)
dq=leastSquares(J,position-r0,1e-6);
q=q0+dq;
end

function dq=leastSquares(A,b,threshold)
% Least Squares
[U,S,V]=svd(A);
s=diag(S);
[m,n]=size(S);
Di=zeros(n,m);
d=1./s;
Di(1:m,1:m)=diag(d);
dq=V*Di*U'*b;
end

function dq=SDLS2(J,e,threshold)
% Selectively Damped Least Squares
% Use clamping to damp motion perpendicular to the goal direction
% Implemented with new method worked out...hopefully this is better
[U,S,V]=svd(J);
s=diag(S);
[m,n]=size(S);
k=floor(m/3);

d=zeros(1,min(m,n));
ind=find(s>=threshold);
d(ind)=1./s(ind);

U3=reshape(U.^2,3,m,k);
N=sum(sqrt(sum(U3,1)),3);

% find effective V
Jtemp=reshape(J.^2,3,n,k);
rho=sqrt(sum(Jtemp,1));
M=zeros(1,m);

for l=1:k
    M=M+rho(:,:,l)*abs(V(:,1:m)).*d;
end

gamma=pi/4*min([ones(1,m);N./M]);

Di=zeros(n,m);
Di(ind,ind)=diag(d(ind));

dq=clampMaxAbs(V.*repmat((Di*U'*e)',n,1),gamma);
end
