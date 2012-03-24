%% Show a demonstration of basic arm movements using the MiniHubo IK solver

%% 1 - Define the path of each arm as a sequence of motions between
%% keyframes

dt=.1;
leftArmFrames=[0 20 40 70;
               0 0 0 40;
               10 10 10 10];
leftArmTimes=[0 1 2 5]


% rightArmFrames=[10 10 10 0;
%                10 30 20 0;
%                10 0  10 20];
rightArmFrames=zeros(3,4);
rightArmTimes=[0 .5 2.5 5]




%% Process Data into individual positions
[leftArmX,leftArmY,leftArmZ]=interpolateArm(leftArmTimes,leftArmFrames,dt)
[rightArmX,rightArmY,rightArmZ]=interpolateArm(rightArmTimes,rightArmFrames,dt)
% leftArmX=50:-1:0;
N=length(leftArmX)
% leftArmY=zeros(N,1);
% leftArmZ=zeros(N,1);
% 
% rightArmX=zeros(N,1);
% rightArmY=zeros(N,1);
% rightArmZ=zeros(N,1);

% Write trajectories out to disk to save memory
mkdir('trajectories');
fid=fopen('trajectories\trajectory1.txt','wt');

fprintf(fid,'%f ',zeros(1,N));fprintf(fid,'\n');
fprintf(fid,'%f ',zeros(1,N));fprintf(fid,'\n');
fprintf(fid,'%f ',zeros(1,N));fprintf(fid,'\n');
fprintf(fid,'%f ',zeros(1,N));fprintf(fid,'\n');
fprintf(fid,'%f ',zeros(1,N));fprintf(fid,'\n');
fprintf(fid,'%f ',zeros(1,N));fprintf(fid,'\n');
fprintf(fid,'%f ',leftArmX);fprintf(fid,'\n');
fprintf(fid,'%f ',leftArmY);fprintf(fid,'\n');
fprintf(fid,'%f ',leftArmZ);fprintf(fid,'\n');
fprintf(fid,'%f ',rightArmX);fprintf(fid,'\n');
fprintf(fid,'%f ',rightArmY);fprintf(fid,'\n');
fprintf(fid,'%f ',rightArmZ);fprintf(fid,'\n');
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

%% IK Solution and export (basically ignore this if you don't care about the details of the implementation)
% Initialize variables for IK solver
%Define a safe initial position in joint angles
initialPose=[0 0 0 pi/6 pi/3 pi/6 0 0 0 pi/6 pi/3 pi/6 0 pi/8 0 pi/8 pi/2 pi/8 0 pi/8 pi/2]';
% Get the resulting initial positions of all limbs
initialPosition=miniForwardKinematics(initialPose);
k=1;

fclose all;

outputFileName='test';
[angleFID,MESSAGE]=fopen([outputFileName,'.txt'],'w');
%There are some issues with the inital few points, so this is a hack to
%increase the number of iterations on the first point
qStart=miniIKSolve(trajectory(:,1)+initialPosition,initialPose);
n=floor(1/dt);
dq=(qStart-initialPose)/n;
%Write the initial pose startup to the file by simple linear interpolation
for j=1:n
    fprintf(angleFID,'%f ',initialPose+dq*j);
    fprintf(angleFID,'%f ',0);
    fprintf(angleFID,'\n');
end

%Begin solution of IK in steps,writing the results each loop to the output
%file.  When all trajectories have been solved, close the file. 
while k<size(trajectory,2)
    goalPosition=trajectory(:,k)+initialPosition;
    angles=miniIKSolve(goalPosition,qStart);
    fprintf(angleFID,'%f ',angles);
    fprintf(angleFID,'%f ',0);
    fprintf(angleFID,'\n');
    k=k+1;
	qStart=angles;
end

fclose(angleFID);
outputData=importdata([outputFileName,'.txt']);

%profile off
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
animateSolution(outputData(n+1:end,1:21)',[],1/dt,IKSolver,[],[-150 150 -200 200 -robot.COMHeight robot.COMHeight*1.5])
%stepAnimateSolution(outputData(A0:A1,1:21)',trajectory(19:24,A0:A1),1/robot.dt,IKSolver,[],[-100 1200 -300 300 0 1200])
%profile viewer
save(outputFileName,'robot'); 

