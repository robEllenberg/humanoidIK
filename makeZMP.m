function [xZMP,yZMP,t]=makeZMP(leftPath,rightPath,thetaLeft,thetaRight,leftStepLengths,rightStepLengths,robot)
global DEBUG
% Standard indices give row numbers corresponding to X,Y,Z
% Standard vector storage as column vectors, with column # as "time" or
% whatever index
% [ x0 x1 ... xn ]
% [ y0 y1 ... yn ]
% [ z0 z1 ... zn ]
X=1;
Y=2;
Z=3;

%Hard code optimal parameters for now, make these part of a structure or
%something later

pdsX=robot.doubleSupportRatio;
pdsY=robot.doubleSupportRatio; %hack this to get variation, eventually make this a parameter
dt=robot.dt;
%Trim single and double support times to multiples of dt
tStep=floor(robot.tStep/dt)*dt;
tdsX=floor(pdsX*tStep/dt)*dt*2;
tdsY=floor(pdsY*tStep/dt)*dt*2;
% Use sanitized values to produce time offset vectors for ss and ds
dsTVecX=dt:dt:tdsX;
ssTVecX=(tdsX+dt):dt:tStep;

dsTVecY=dt:dt:tdsY;
ssTVecY=(tdsY+dt):dt:tStep;

ldsX=length(dsTVecX);
lssX=length(ssTVecX);

ldsY=length(dsTVecY);
lssY=length(ssTVecY);

%% ZMP Trajectory building
%%TODO: Create better documentation here, and refactor to eliminate all the
%%duplicate code!
%1) Create initial dead zone to start the trajectory stably
t=0:dt:2*tStep-tdsX/2;
xZMP=ones(size(t)).*(leftPath(X,1)+rightPath(X,1))/2;
yZMP=ones(size(t)).*(leftPath(Y,1)+rightPath(Y,1))/2;
%2) Translate the individual step vectors into a single x ZMP distance
%vector
% Lind=(1:length(leftPath))*2-1;
% Rind=(1:length(rightPath))*2;
Lind=(1:length(leftPath))*2-1;
Rind=(1:length(rightPath))*2;

xPoints(Lind)=leftPath(X,:);
xPoints(Rind)=rightPath(X,:);

yPoints(Lind)=leftPath(Y,:);
yPoints(Rind)=rightPath(Y,:);
%yPoints(1)=yPoints(1)*40/50;
%yPoints(2)=yPoints(2)*48/50;

footY=robot.footWidth/2+robot.footSpacing/2;

%% Compensate beginning and end Y vector
% Shrink lateral direction motion to avoid tipping over
yStartVec=leftPath(:,1)-rightPath(:,1);
yStartDir=yStartVec/norm(yStartVec);

yPoints(1)=yPoints(1)-yStartDir(2)*footY/2;
yPoints(2)=yPoints(2)+yStartDir(2)*footY*.1;
xPoints(1)=xPoints(1)-yStartDir(1)*footY/2;
xPoints(2)=xPoints(2)+yStartDir(1)*footY*.1;

yEndVec=leftPath(:,end)-rightPath(:,end);
yEndDir=yEndVec/norm(yEndVec);

yPoints(end-1)=yPoints(end-1)-yEndDir(2)*footY*.1;
yPoints(end)=yPoints(end)+yEndDir(2)*footY/2;
xPoints(end-1)=xPoints(end-1)-yEndDir(1)*footY*.1;
xPoints(end)=xPoints(end)+yEndDir(1)*footY/2;

%Startup step (look into breaking this into a function later?)
% Figure out how to make this without growing inside a loop later
%size(zeros(1,(ldsX+lssX)*(length(xPoints)+4)))
dx=(xPoints(1)-xZMP(end))/ldsX;
    dy=(yPoints(1)-yZMP(end))/ldsY;
    %xZMP(end)+cumsum(ones(1,ldsX)*dx)
    %yZMP(end)+cumsum(ones(1,ldsY)*dy)
    %xZMP(end):dx:xPoints(1)
    xZMP=[xZMP,xZMP(end)+cumsum(ones(1,ldsX)*dx)];
    yZMP=[yZMP,yZMP(end)+cumsum(ones(1,ldsY)*dy)];

    xZMP=[xZMP,ones(1,lssX)*xPoints(1)];
    yZMP=[yZMP,ones(1,lssY)*yPoints(1)];
t=[t,dsTVecX+t(end),ssTVecX+t(end)];
%loop through and make 1 cycle at a time of ZMP
for k=2:length(xPoints)
    dx=(xPoints(k)-xPoints(k-1))/ldsX;
    dy=(yPoints(k)-yPoints(k-1))/ldsY;
    xZMP=[xZMP,xZMP(end)+cumsum(ones(1,ldsX)*dx)];
    yZMP=[yZMP,yZMP(end)+cumsum(ones(1,ldsY)*dy)];

    xZMP=[xZMP,ones(1,lssX)*xPoints(k)];
    yZMP=[yZMP,ones(1,lssY)*yPoints(k)];
    t=[t,dsTVecX+t(end),ssTVecX+t(end)];
end

% Stand still at end of trajectory
endX=(leftPath(X,end)+rightPath(X,end))/2;
endY=(leftPath(Y,end)+rightPath(Y,end))/2;
dx=(endX-xZMP(end))/ldsX;
    dy=(endY-yZMP(end))/ldsY;
    xZMP=[xZMP,xZMP(end)+cumsum(ones(1,ldsX)*dx)];
    yZMP=[yZMP,yZMP(end)+cumsum(ones(1,ldsY)*dy)];

    xZMP=[xZMP,ones(1,lssX)*endX];
    yZMP=[yZMP,ones(1,lssY)*endY];
Ts=[dsTVecX,ssTVecX];
t=[t,Ts+t(end)];

for k=1:ceil(1.5/robot.tStep)
     xZMP=[xZMP,ones(1,lssX+ldsX)*endX];
     yZMP=[yZMP,ones(1,lssY+ldsX)*endY];
     t=[t,Ts+t(end)];
end

%Convention established in smoothing function gives left foot first step:

end


