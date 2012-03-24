function [leftPath,rightPath,thetaLeft,thetaRight,leftStepLengths,rightStepLengths]=smoothPath(pathX,pathY,robot)
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

dpathX=diff(pathX);
dpathY=diff(pathY);
%theta=atan2(dpathX,dpathY);
arcLengths=[0,sqrt(dpathX.^2+dpathY.^2)];
distance=cumsum(arcLengths);
xp=csape(distance,pathX,'variational');
yp=csape(distance,pathY,'variational');

sf=sum(arcLengths);
%NOTE: This part may not be necessary if A* chooses good paths.  if not, then we
%need to pull out key points from the path and use the spline to round off
%the corners.
t=0:sf/60:sf;
splineX=fnval(xp,t);
splineY=fnval(yp,t);
dX=diff(splineX);
dY=diff(splineY);
ds=sqrt(dX.^2+4*dY.^2);%inflate Y distance to reduce sidestepping
s=cumsum([0,ds]);

%% Generate offset paths
footOffset=robot.footWidth/2+robot.footSpacing/2;%dummy units for now..fix this
%offsetDirections=[-dY,-dY(end);dX,dX(end)]; %Left offset is positive, right offset simply opposite of this
offsetDirections=[zeros(1,length(splineX));ones(1,length(splineX))]; %Left offset is positive, right offset simply opposite of this
offsetVectors=offsetDirections./repmat(sqrt(sum((offsetDirections).^2,1)),2,1)*footOffset;

rightFootPathX=splineX-offsetVectors(X,:);
rightFootPathY=splineY-offsetVectors(Y,:);
leftFootPathX=splineX+offsetVectors(X,:);
leftFootPathY=splineY+offsetVectors(Y,:);

dlfX=diff(leftFootPathX);
dlfY=diff(leftFootPathY);
dsL=sqrt(dlfX.^2+dlfY.^2);
sL=cumsum([dsL,dsL(end)]);

drfX=diff(rightFootPathX);
drfY=diff(rightFootPathY);
dsR=sqrt(drfX.^2+drfY.^2);
sR=cumsum([dsR,dsR(end)]);

%% Calculate maximum possible hip movement
% Want to break up hip average path into segments of as large a movement as
% possible. 
% A robust way would be to calculate the radius of curvature locally, then
% figure out the turning angle and resulting step length difference.  This
% requires 2nd difference, which leaves annoying end conditions since the
% resolution is so coarse.

% Hack it instead: Pick a small enough step length, check via interpolation
% if it's too big a step, and repeat until it works.
nomStep=robot.stepSize;
goalDist=s(end);
steps=ceil(goalDist/(nomStep))*2+1;
hipX=0:goalDist/steps:goalDist;

% Overtaking Steps
leftInd=hipX([1,2:2:end]);
rightInd=hipX([1:2:end,end]);

% Matching steps
% leftInd=hipX([1,2:2:end]);
% rightInd=hipX([1,2:2:end]);

dLeft=interp1(s(1:end-1)',[dlfX;dlfY]',leftInd,'linear','extrap')';
dRight=interp1(s(1:end-1)',[drfX;drfY]',rightInd,'linear','extrap')';
thetaLeft=atan2(dLeft(Y,:),dLeft(X,:))*0;
% Limit to positive angle difference between left and right)
%thetaRight=min(thetaLeft([2:end end]),atan2(dRight(Y,:),dRight(X,:)));
thetaRight=atan2(dRight(Y,:),dRight(X,:))*0;
%perpVec=[-sin(thetaLeft/2+thetaRight/2);cos(thetaLeft/2+thetaRight/2)];

leftPath=interp1(s',[leftFootPathX;leftFootPathY]',leftInd)';
rightPath=interp1(s',[rightFootPathX;rightFootPathY]',rightInd)';

Lind=floor((1:length(hipX)+1)/2+.4)+1;
Rind=floor((1:length(hipX)+1)/2-.4)+1;
chosenHipPath=(leftPath(:,Lind)+rightPath(:,Rind))/2;
footOffset=robot.footSpacing+robot.footWidth;
tempRight=min([rightPath(Y,:);leftPath(Y,:)-footOffset;leftPath(Y,[2:end,end])-footOffset]);

offsets=max(max(leftPath(Y,[2:end,end])-tempRight,leftPath(Y,:)-tempRight),footOffset)/(footOffset);
% New: For torso always facing forwards, simple offset sharing:
leftPath(Y,:)=offsets.*(leftPath(Y,:)-chosenHipPath(Y,[1,2:2:end]))+chosenHipPath(Y,[1,2:2:end]);
rightPath(Y,:)=offsets.*(rightPath(Y,:)-chosenHipPath(Y,[1,2:2:end]))+chosenHipPath(Y,[1,2:2:end]);

%Calculate food angles at each point

%NOTE: these commented parts are transposed and need to be rewritten if
%used
%leftDistance=interp1(s',sL',hipX);
%temp=diff([0,leftDistance]);
%leftSteps=sum(reshape(temp,2,length(hipX)/2),1);

%rightDistance=interp1(s',sR',hipX);
%temp=diff([0,rightDistance]);
%rightSteps=sum(reshape(temp,2,length(hipX)/2),1);

%actHipX=[hipX(1),hipX(1:end-1)+goalDist/steps/2,hipX(end)];
%centerPath=interp1(s',[splineX;splineY]',actHipX);
%calculate indices to find hip positions from foot positions


%Check values to make sure they don't exceed the maximum step
leftStepLengths=sqrt(sum(diff(leftPath,[],2).^2,1));
rightStepLengths=sqrt(sum(diff(rightPath,[],2).^2,1));
% TODO: turning radius and angle constraint somewhere in here so that the
% feet don't have a 

if DEBUG
% figure(1)
% fnplt(xp)
% axis equal
% figure(2)
% fnplt(yp)
% axis equal
% figure(3)
% plot(splineX,splineY,pathX,pathY)
% axis equal
figure('Name','Walking Path')
plot(splineX,splineY,leftFootPathX,leftFootPathY,...
    rightFootPathX,rightFootPathY,...
    leftPath(X,:),leftPath(Y,:),'g*',...
    rightPath(X,:),rightPath(Y,:),'r*',...
    chosenHipPath(X,:),chosenHipPath(Y,:),'c+')
axis equal
grid on
title('Spline interpolated path of hip, with offset paths for feet')
legend('Spline Hip trajectory','offset left','offset right','left Foot steps','right Foot steps','Hip positions')
end

end
