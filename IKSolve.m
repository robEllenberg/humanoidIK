function angles=IKSolve(unscaledPositions,q0,~,IKSolver)
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
positions=unscaledPositions;
n=IKSolver.size(2);
[m,points]=size(positions);

IKSolver.method=@leastSquares;
IKSolver.skipConvergenceError=1;
tol=.0005;

angles=zeros(n,points);
angles(:,1)=solutionStep(positions(:,1),q0,[],IKSolver);
actualPositions=zeros(m,points);
actualPositions(:,1)=IKSolver.forwardKinematics(angles(:,1));
tempangles=angles(:,1);
k=1;
j=0;
abortSolution=0;
tic;

while k<points && ~abortSolution
    tempangles=solutionStep(positions(:,k+1),tempangles,[],IKSolver);
    temppositions=IKSolver.forwardKinematics(tempangles);
    j=j+1; 
    positionError=norm(temppositions-positions(:,k+1));
%     if ~ishandle(h)
%         abortSolution=1;
%     elseif positionError<tol
    if positionError<tol
        angles(:,k+1)=tempangles;
        actualPositions(:,k+1)=temppositions;
%         if mod(k,20)==0
%             waitbar(k/points,h,sprintf('Step %d took %d iterations; elapsed time is %.2f sec.',k,j,toc))
%         end
        % Update solution variables
        k=k+1;j=0;
    elseif j>30
        %solution failure at timestep, use previous value and increment
        %(since there may be a future position that works)
        if IKSolver.skipConvergenceError
            
            angles(:,k+1)=angles(:,k);
            actualPositions(:,k+1)=actualPositions(:,k);
            %waitbar(k/points,h,sprintf('Step %d failed at %d iterations; elapsed time is %.2f sec.',k,j,toc))
            fprintf('Failure at step %d\n',k)
            k=k+1;j=0;
        else
            abortSolution=1;
        end
    end   
end
if abortSolution
    %Use the good data and quit
    k=max(k-1,1);
    %actualPositions=actualPositions(:,1:k);
    angles=angles(:,1:k);
end
% if ishandle(h)
%     close(h)
% end
%Use handle class to store solution for future use
end

function dq=clampMaxAbs(Psi,gamma)
[~,n]=size(Psi);
Gamma=[repmat(gamma,n,1),zeros(n,n-length(gamma))];
%fugly way to do ..
qtemp=(abs(Psi)<=Gamma).*Psi+(abs(Psi)>Gamma).*Gamma.*sign(Psi);
dq=sum(qtemp,2);
end

function q=solutionStep(position,q0,~,IKSolver)
% Single iteration of least squares solution
J=IKSolver.Jacobian(q0);
r0=IKSolver.forwardKinematics(q0);
% Change here for different methods (crude)
dq=IKSolver.method(J,position-r0,1e-6);
q=q0+dq;
end

function dq=leastSquares(A,b,threshold)
% Least Squares
[U,S,V]=svd(A);
s=diag(S);
[m,n]=size(S);
Di=zeros(n,m);
ind=find(diag(S)>=threshold);
d=zeros(n,1);
d(ind)=1./s(ind);
Di(ind,ind)=diag(d(ind));
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

function [p0,q0]=initialConditions(IKSolver)
%TODO: Define initial conditions for walk here (NOT the initial
%conditions for the solver
q0=[0 0 0 pi/6 pi/3 pi/6 0 0 0 pi/6 pi/3 pi/6 0 pi/8 0 pi/8 pi/2 pi/8 0 pi/8 pi/2]';
p0=IKSolver.forwardKinematics(q0,[0 0 0 0 0 0]);
p0=p0+p0(3); %offset so that foot is at zero height
end