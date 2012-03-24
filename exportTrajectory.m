%% General Walk Trajectory for Hubo
% By Robert Ellenberg & Youngbum Jun
% 
% Function implementation to take a charted path, spline interpolate it,
% and produce the complete walking trajectory needed for the mini-Hubo to
% walk.
%

%% Initialize tuning parameters
global DEBUG
DEBUG=0;
%profile on
% standard units are mmks for these numbers
robot.rollBias=3*pi/180;
robot.footWidth=64;
robot.footSpacing=20;
robot.stepSize=100;
robot.dt=.02;
robot.zc=.160;
robot.COMHeight=220;
robot.doubleSupportRatio=.2;
robot.stepHeight=30;
robot.tStep=.4;
robot.prefix='rob_fast_astar';

k=3;
% ftest=0;
% while ftest>=0
%     k=k+1;
%     ftest=fopen([robot.prefix,sprintf('%d',k)],'r');
% end
outputFileName=sprintf('trajectories\\%s%d',robot.prefix,k)

%% Generate a walking path based on a few key points
[roughX,roughY]=findAstarPath_png([100,600],[2400,400]);

% Undersample the rough path from A* to do crude smoothing, then feed the
% resulting points to the spline interpolator
nRough=length(roughX);
xStart=interp1(1:nRough,roughX',1:nRough/10:nRough);
yStart=interp1(1:nRough,1.05*roughY',1:nRough/10:nRough);

[leftPath,rightPath,thetaLeft,thetaRight,leftStepLengths,rightStepLengths]=smoothPath(xStart,yStart,robot);

[xZMP,yZMP,t]=makeZMP(leftPath,rightPath,thetaLeft,thetaRight,leftStepLengths,rightStepLengths,robot);

[Gp,Ke,Kx,sys]=previewGains(robot.dt,robot.zc);

%% Generate Hip trajectory
[CoMf,CoMl]=hipPreviewTrajectory(xZMP,yZMP,t,robot);

%% Calculate foot trajectory to match
[xL,yL,zL,xR,yR,zR,thetaL,thetaR]=footTrajectory(leftPath, rightPath,thetaLeft,thetaRight,robot);
thetaT=(thetaR+thetaL)/2;
%N=length(xL);
N=min(length(xZMP),length(xL));
if DEBUG
	figure(6)
	plot(t(1:N),xL(1:N),t(1:N),CoMf(1:N),t(1:N),xR(1:N),t(1:N),xZMP(1:N),'--')
	title('x axis trajectories of foot and hip')
	legend('left','hip','right','xZMP')
	xlabel('Time,seconds')
	ylabel('milimeters')

	figure(7)
	plot(t(1:N),yL(1:N),t(1:N),CoMl(1:N),t(1:N),yR(1:N),t(1:N),yZMP(1:N),'--','LineWidth',2)
	title('Y axis trajectories of foot and hip')
	xlabel('Time,seconds')
	ylabel('milimeters')
	legend('left','hip','right','yZMP')

	figure(8)
	plot(t(1:N),thetaL(1:N),t(1:N),thetaR(1:N),t(1:N),thetaT(1:N))
	title('Yaw of feet')
	xlabel('Time,seconds')
	ylabel('radians')
	legend('Left','Right','Torso')
end
disp('Starting IK Solution....')

S=1;
armX=10*ones(1,N-S+1).*cos(thetaT(S:N))+100*ones(1,N-S+1).*sin(thetaT(S:N));
armY=-10*ones(1,N-S+1).*sin(thetaT(S:N))+100*ones(1,N-S+1).*cos(thetaT(S:N));

mkdir('trajectories');
fid=fopen('trajectories\trajectory1.txt','w');
zh=ones(1,N-S+1)*robot.COMHeight;
fprintf(fid,'%f ',xL(S:N)-CoMf(S:N));fprintf(fid,'\n');
fprintf(fid,'%f ',yL(S:N)-CoMl(S:N));fprintf(fid,'\n');
fprintf(fid,'%f ',zL(S:N)-zh);fprintf(fid,'\n');
fprintf(fid,'%f ',xR(S:N)-CoMf(S:N));fprintf(fid,'\n');
fprintf(fid,'%f ',yR(S:N)-CoMl(S:N));fprintf(fid,'\n');
fprintf(fid,'%f ',zR(S:N)-zh);fprintf(fid,'\n');
fprintf(fid,'%f ',-armX);fprintf(fid,'\n');
fprintf(fid,'%f ',armY);fprintf(fid,'\n');
fprintf(fid,'%f ',zeros(1,N-S+1));fprintf(fid,'\n');
fprintf(fid,'%f ',armX);fprintf(fid,'\n');
fprintf(fid,'%f ',-armY);fprintf(fid,'\n');
fprintf(fid,'%f ',zeros(1,N-S+1));fprintf(fid,'\n');
fclose(fid);

%Load the solver structure, recreate it if it doesn't exist
try
    load miniIKSolver IKSolver
catch ME
    disp('IKSolver file not found') %figure out exceptions at some point
    clear IKSolver
    IKSolver=makeIKSolver('miniSkeleton.mat','mini',[0,0]);
end 
trajectory=load('trajectories\trajectory1.txt');

if DEBUG
    figure(9)
    plot(trajectory(1:6,:)')
    title('Exported trajectory inputs')
    legend('left foot x','left foot y','left foot z','right foot x','right foot y','right foot z')
    
    figure(10)
    plot(diff(trajectory'))
    title('Exported trajectory 1st order differences')
end
%% IK Solution and export
% Initialize variables for IK solver

q0=[0 0 0 pi/6 pi/3 pi/6 0 0 0 pi/6 pi/3 pi/6 0 pi/8 0 pi/8 pi/2 pi/8 0 pi/8 pi/2]';
k=1;

fclose all;
[angleFID,MESSAGE]=fopen([outputFileName,'.txt'],'w');
%There are some issues with the inital few points, so this is a hack to
%increase the number of iterations on the first point
qStart=miniIKSolve(trajectory(:,1),q0);
n=floor(1/robot.dt);
dq=(qStart-q0)/n;
%Write the initial pose startup to the file by simple linear interpolation
for j=1:n
    fprintf(angleFID,'%f ',q0+dq*j);
    fprintf(angleFID,'%f ',0);
    fprintf(angleFID,'\n');
end

%Begin solution of IK in steps,writing the results each loop to the output
%file.  When all trajectories have been solved, close the file. 
while k<size(trajectory,2)
    goalPosition=trajectory(:,k);
    angles=miniIKSolve(goalPosition,qStart);
    fprintf(angleFID,'%f ',angles);
    fprintf(angleFID,'%f ',0);
    fprintf(angleFID,'\n');
    k=k+1;
	qStart=angles;
end

fclose(angleFID);
outputData=importdata([outputFileName,'.txt']);
A0=600;
A1=1100;
%profile off
animateSolution(outputData(n+1:end,1:21)',[],1/robot.dt,IKSolver,[],[-150 150 -200 200 -robot.COMHeight robot.COMHeight*1.5])
%stepAnimateSolution(outputData(A0:A1,1:21)',trajectory(19:24,A0:A1),1/robot.dt,IKSolver,[],[-100 1200 -300 300 0 1200])
%profile viewer
save(outputFileName,'robot'); 
