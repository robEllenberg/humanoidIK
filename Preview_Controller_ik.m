%% Preview Controller script
% For exporting solved angles for the given hand-coded ZMP trajectory.
% See IKSolve.m's help for issues with the IK solver
clear all
close all
zc = 0.23;
g = 9.81;

hd = 75;
ld = 30;

delt = 0.033;

% Time sequnce (7 steps)
x0 = 0:delt:0.9;
x1 = (0.9+delt):delt:1.1;
x2 = (1.1+delt):delt:1.9;
x3 = (1.9+delt):delt:2.1; % the right foot should land at 2 seconds
x4 = (2.1+delt):delt:2.9; % the left foot should take off at 2.1 seconds
x5 = (2.9+delt):delt:3.1; % the right foot should land at 3 seconds
x6 = (3.1+delt):delt:3.9; % the left foot takes off at 3.1 sec
x7 = (3.9+delt):delt:4.1; % the left foot lands at 4 sec
x8 = (4.1+delt):delt:4.9; % the right foot takes of at 4.1 sec
x9 = (4.9+delt):delt:5.1;
x10 = (5.1+delt):delt:5.9;
x11 = (5.9+delt):delt:6.1;
x12 = (6.1+delt):delt:6.9;
x13 = (6.9+delt):delt:7.1;
x14 = (7.1+delt):delt:7.9;
x15 = (7.9+delt):delt:8.1;
x16 = (8.1+delt):delt:8.9;
x17 = (8.9+delt):delt:9.1;
x18 = (9.1+delt):delt:11.1;

x = [x0 x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 x13 x14 x15 x16 x17 x18];

% Desired ZMP in forward direction
y0 = zeros(1,length(x0));
y1 = zeros(1, length(x1));
y2 = zeros(1, length(x2));
y3 = linspace(0,ld,length(x3));
y4 = ld*ones(1,length(x4));
y5 = linspace(ld,2*ld,length(x5));
y6 = 2*ld*ones(1,length(x6));
y7 = linspace(2*ld,3*ld,length(x7));
y8 = 3*ld*ones(1,length(x8));
y9 = linspace(3*ld,4*ld,length(x9));
y10 = 4*ld*ones(1,length(x10));
y11 = linspace(4*ld,5*ld,length(x11));
y12 = 5*ld*ones(1,length(x12));
y13 = linspace(5*ld,6*ld,length(x13));
y14= 6*ld*ones(1,length(x14));
y15 = linspace(6*ld,7*ld,length(x15));
y16 = 7*ld*ones(1,length(x16));
y17 = 7*ld*ones(1,length(x17));
y18 = 7*ld*ones(1,length(x18));

ydf = [y0 y1 y2 y3 y4 y5 y6 y7 y8 y9 y10 y11 y12 y13 y14 y15 y16 y17 y18];

% ZMP in lateral direction

y0 = zeros(1, length(x0));% 0 - 0.9
y1 = linspace(0,hd,length(x1)); % 0.9+delt - 1.1
y2 = hd*ones(1,length(x2)); % 1.1+delt - 1.9
y3 = linspace(hd, -hd,length(x3));% 1.9+delt - 2.1
y4 = -hd*ones(1,length(x4)); % 2.1+delt - 2.9
y5 = linspace(-hd,hd,length(x5)); % 2.9+delt - 3.1
y6 = hd*ones(1,length(x6)); % 3.1+delt - 3.9
y7 = linspace(hd,-hd,length(x7)); % 3.9+delt - 4.1
y8 = -hd*ones(1,length(x8)); % 4.1+delt - 4.9
y9 = linspace(-hd,hd,length(x9)); % 4.9+delt - 5.1
y10 = hd*ones(1,length(x10)); % 5.1+delt - 5.9
y11 = linspace(hd,-hd,length(x11)); % 5.9+delt - 6.1
y12 = -hd*ones(1,length(x12)); % 6.1+delt - 6.9
y13 = linspace(-hd,hd,length(x13)); % 6.9+delt - 7.1
y14 = hd*ones(1,length(x14)); % 7.1+delt - 7.9
y15 = linspace(hd,-hd,length(x15)); % 7.9+delt - 8.1
y16 = -hd*ones(1,length(x16)); % 8.1+delt - 8.9
y17 = linspace(-hd,0,length(x17)); %
y18 = 0*ones(1,length(x18));

ydl = [y0 y1 y2 y3 y4 y5 y6 y7 y8 y9 y10 y11 y12 y13 y14 y15 y16 y17 y18];

% optimal gain refered to preview H2 theory 
% State-Space representation
[Gp,Ke,Kx,sys]=previewGains(delt,zc);
N=1:length(Gp);
A=sys.A;
B=sys.B;
C=sys.C;

xkf = zeros(3,length(x)); % state
xkl = zeros(3,length(x));
ukf = zeros(1,length(x)); % Control input
ukl = zeros(1,length(x));

ykf = zeros(1,length(x)); % forward direction
ykl = zeros(1,length(x)); % lateral direction
CoMf = zeros(1,length(x));
CoMl = zeros(1,length(x));
ef = 0; % summation of error
el = 0;
for k = 1:length(x)
    
    if (k+length(N))<length(x) 
        
        CoMf(k) = xkf(1,k);
        CoMl(k) = xkl(1,k);
        
        ykf(k) = C*xkf(:,k); % C = output matrix, xk = 
        ykl(k) = C*xkl(:,k);
        
        ef = ef + ykf(k) - ydf(k);
        el = el + ykl(k) - ydl(k);
        prevf = 0;
        prevl = 0;
        
        for i = 1:length(N)
            if (k+length(N))<length(x) 
                prevf = prevf + 1*Gp(i)*ydf(k+i);
                prevl = prevl + 1*Gp(i)*ydl(k+i);
            end        
        end
        
    else
        ykf(k) = ykf(k-1);  % there is no enough preview point
        ykl(k) = ykl(k-1);
        CoMf(k) = xkf(1,k-1);
        CoMl(k) = xkl(1,k-1);
    end
    
    ukf(k) = -Ke*ef - Kx*xkf(:,k) - prevf;
    ukl(k) = -Ke*el - Kx*xkl(:,k) - prevl;
    xkf(:,k+1) = A*xkf(:,k)+B*ukf(k);
    xkl(:,k+1) = A*xkl(:,k)+B*ukl(k);
end
% 

figure(1)

plot(N,Gp,'linewidth',3);
title('Preview Gain, Gp, vs. Time');
xlabel('Time (second)');
ylabel('Magnitude of Gp');
legend('Gp');
grid on

figure(2)
plot(x,ydf,'b','linewidth',3);grid on
hold on
plot(x,ykf,'r-.','linewidth',4)
hold on
plot(x,CoMf,'g--','linewidth',3);
title('Desired ZMP, ZMP via Preview, and CoM via Preview');
xlabel('Time (second)');
ylabel('Distance in Forward Direction (m)');
legend('ZMP ref.','ZMP Preview','CoM Preview');
hold off

figure(3)

plot(x,ydl,'b','linewidth',3);grid on
hold on
plot(x,ykl,'r-.','linewidth',4)
hold on
plot(x,CoMl,'g--','linewidth',3);
title('Desired ZMP, ZMP via Preview, and CoM via Preview');
xlabel('Time (second)');
ylabel('Distance in Lateral Direction (m)');
legend('ZMP ref.','ZMP Preview','CoM Preview');
hold off


figure(5)
plot(x,ydl,'linewidth',3);
title('ZMP in Lateral');
xlabel('Time (second)');
ylabel('Step Distance');
ylim([-8 8])
legend('ZMP in Y');
grid on
hold off

xl = [];
yl = [];
zl = [];

xr = [];
yr = [];
zr = [];

% right foot x direction
for t=0:delt:(1.1-delt)
    
    xr = [xr, 0];
    yr = [yr, 40];
    zr = [zr, 0];
end
for t=0:delt:2
    [a, b, c] = foot(t, 2, 0.8, ld, 40, 40, 0, 0, 0 );
    xr = [xr, a];
    yr = [yr, b];
    zr = [zr, c];
end
for t=0:delt:(6-delt)
    [a, b, c] = foot(t, 2, 0.8, 2*ld, 40, 40, 0, ld, 0 );
    xr = [xr, a];
    yr = [yr, b];
    zr = [zr, c];
end

% left foot x direction
for t=0:delt:(2.1-delt)
    
    xl = [xl, 0];
    yl = [yl, 40];
    zl = [zl, 0];
end
for t=0:delt:6
    [a, b, c] = foot(t, 2, 0.8, 2*ld, 40, 40, 0, 0, 0 );
    xl = [xl, a];
    yl = [yl, b];
    zl = [zl, c];
end
offset = xl(end);
for t=0:delt:(1-delt)
    [a, b, c] = foot(t, 2, 0.8, ld, 40, 40, 0, offset, 0 );
    xl = [xl, a];
    yl = [yl, b];
    zl = [zl, c];
end
points=length(ykf);
for t=1:points-length(xl)
    
    xl = [xl, xl(end)];
    yl = [yl, yl(end)];
    zl = [zl, zl(end)];
	xr = [xr, xr(end)];
    yr = [yr, yr(end)];
    zr = [zr, zr(end)];
end

figure(15)
plot(x,ydf,'linewidth',1);
hold on
plot(x, [xr;zr;xl;zl;CoMf;CoMl]);
title('ZMP in Forward');
xlabel('Time (second)');
ylabel('Step Distance');
% ylim([-8 60])
legend('ZMP in X','R foot in x','R foot in z','L foot in x','L foot in z','CoM in x','CoM in y');
grid on
hold off
% 
figure(4)
plot(x,ydf,'linewidth',1);
hold on

plot(x, [xr;zr;xl;zl;CoMf;CoMl]);
title('ZMP in Forward');
xlabel('Time (second)');
ylabel('Step Distance');
ylim([-8 60])
legend('ZMP in X','R foot in x','R foot in z','L foot in x','L foot in z','CoM in x','CoM in y');
grid on
hold off

% Build trajectory arrays in format usable by the IK solver


%Write trajectory source file.  Foot and arm trajectories are defined
%relative to torso, hence the subtraction
fid=fopen('trajectory1.txt','w');
zh=ones(size(ykf))*zc*1000;
fprintf(fid,'%f ',xl-CoMf);fprintf(fid,'\n');
fprintf(fid,'%f ',yl-CoMl);fprintf(fid,'\n');
fprintf(fid,'%f ',zl-zh);fprintf(fid,'\n');
fprintf(fid,'%f ',xr-CoMf);fprintf(fid,'\n');
fprintf(fid,'%f ',-yr-CoMl);fprintf(fid,'\n');
fprintf(fid,'%f ',zr-zh);fprintf(fid,'\n');
fprintf(fid,'%f ',zeros(1,points));fprintf(fid,'\n');
fprintf(fid,'%f ',zeros(1,points));fprintf(fid,'\n');
fprintf(fid,'%f ',zeros(1,points));fprintf(fid,'\n');
fprintf(fid,'%f ',zeros(1,points));fprintf(fid,'\n');
fprintf(fid,'%f ',zeros(1,points));fprintf(fid,'\n');
fprintf(fid,'%f ',zeros(1,points));fprintf(fid,'\n');
fprintf(fid,'%f ',zeros(1,points));fprintf(fid,'\n');
fprintf(fid,'%f ',100*ones(1,points));fprintf(fid,'\n');
fprintf(fid,'%f ',-10*ones(1,points));fprintf(fid,'\n');
fprintf(fid,'%f ',zeros(1,points));fprintf(fid,'\n');
fprintf(fid,'%f ',-100*ones(1,points));fprintf(fid,'\n');
fprintf(fid,'%f ',-10*ones(1,points));fprintf(fid,'\n');
fclose(fid);

%Load the solver structure, recreate it if it doesn't exist
try
    load IKSolver IKSolver
catch
    IKSolver=IKfunctions;
    save IKSolver IKSolver;
end 
load trajectory1.txt
plot(trajectory1(1:6,:)')
legend('left foot x','left foot y','left foot z','right foot x','right foot y','right foot z')
q0=[0 0 0 pi/6 pi/3 pi/6 0 0 0 pi/6 pi/3 pi/6 0 pi/8 0 pi/8 pi/2 pi/8 0 pi/8 pi/2]';
outputFileName='outputfile7.txt';
fclose all;
[angleFID,MESSAGE]=fopen(outputFileName,'w+');
chunkSize=600;
angletest=IKSolve(trajectory1(:,1:5),q0,IKSolver);

%Output a startup trajectory to gently bring robot into crouch position
startAngles=interp1([0,ceil(2/delt)*delt],[zeros(21,1),angletest(:,2)]',0:delt:ceil(2/delt)*delt)';
for k=1:size(startAngles,2)
   fprintf(angleFID,'%f ',startAngles(:,k));
        fprintf(angleFID,'%f ',0);
        fprintf(angleFID,'\n');
end
figure(11)
%hack to fix the initial step glitch.  Eventuall eliminate this code
%redudndancy
k=0;
angles=IKSolve(trajectory1(:,(k*chunkSize)+1:min((k+1)*chunkSize,points)),q0,IKSolver);
for j=2:size(angles,2)
    fprintf(angleFID,'%f ',angles(:,j));
    fprintf(angleFID,'%f ',0);
    fprintf(angleFID,'\n');
end
k=k+1;
q0=angles(:,end);
while k*chunkSize<size(trajectory1,2)
    angles=IKSolve(trajectory1(:,((k-1)*chunkSize)+1:min((k)*chunkSize,points)),q0,[0,0,zc*1000,0,0,0]',IKSolver);
    for j=1:size(angles,2)
        fprintf(angleFID,'%f ',angles(:,j));
        fprintf(angleFID,'%f ',0);
        fprintf(angleFID,'\n');
    end
    k=k+1;
    q0=angles(:,end);
end

fclose(angleFID)
outputData=importdata(outputFileName);
figure(10)
plot(outputData(:,2:7));
legend('LHY','LHR','LHP','LKP','LAP','LAR')

figure(11)
plot(outputData(:,8:13));
legend('RHY','RHR','RHP','RKP','RAP','RAR')
robot=minihuboIK(IKSolver);
robot.Angles=outputData';
robot.animateSolution(IKSolver,[])
