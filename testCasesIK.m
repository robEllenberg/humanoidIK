load simpleIKSolver
q0=[0 0 0 pi/6 pi/3 pi/6 0 0 0 pi/6 pi/3 pi/6 0 pi/4 0 pi/8 pi/2 pi/4 0 pi/8 pi/2]';
p0=simpleForwardKinematics(q0);
motionScale=10;
angleScale=.1;
% Commanded positions, i.e. the goal position for the robot
fcmd=fopen('testCommands.txt','w');
% Actual positions that the solved angles produce, used to see how close
% the solution is.
fpos=fopen('testPositions.txt','w');
% The resulting angles solved by Ik for each command set
fang=fopen('testAngles.txt','w');
for k=1:100
    dp=rand(18,1).*([motionScale*[1;1;1;1;1;1];angleScale*[1;1;1;1;1;1];motionScale*[1;1;1;1;1;1]]);
    commandPositions=p0+[dp(1:12);zeros(6,1)];
    angles=IKSolveStep(commandPositions,q0,[],IKSolver);
    actualPositions=simpleForwardKinematics(angles);
    fprintf(fpos,'%f ',actualPositions);fprintf(fpos,'\n');
    fprintf(fcmd,'%f ',commandPositions);fprintf(fcmd,'\n');
    fprintf(fang,'%f ',angles);fprintf(fang,'\n');
    if sum(angles==q0)==21
        fprintf('Solution error at line %d\n',k);
    end
end
fclose('all')

load testPositions.txt
load testCommands.txt
%This is a simple check to see there are no gross errors, or if there are
%how frequent they are...
plot(testPositions-testCommands);