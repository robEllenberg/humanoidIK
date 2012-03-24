function [CoMf,CoMl]=hipPreviewTrajectory(xZMP,yZMP,t,robot)
%% Generate Hip trajectory based on foot ZMP in x and y
%  Assumptions: Constant Hip Height, no cornering acceleration (i.e.
%  centripetal accelerations and coriolis terms are neglected), time vector
%  starts at zero
%
% t = input time vector
% xZMP = ZMP trajectory in X direction
% yZMP = ZMP in y direction
% 
% Row vectors are standard here
% if nargin <5
%     %make a time vector the same length as the minimum of the two ZMP
%     %trajectories
%     t=cumsum([0,ones(1,min(length(xZMP,yZMP))-1)/dt]);
% end
global DEBUG
%Input conditioning here (eventually)
dt=robot.dt;
zc=robot.zc;

% Find Preview gains (load if possible to save time)
try
    load storedPreviewGains.mat Gp Ke Kx sys;
catch ME
    %Add better handling code here eventually
    [Gp,Ke,Kx,sys]=previewGains(dt,zc);
    save storedPreviewGains.mat Gp Ke Kx sys;
end
N=1:length(Gp);

xkf = zeros(3,length(t)); % states of preview controller, x1 corresponds to position
xkl = zeros(3,length(t));
ukf = zeros(1,length(t)); % Control input
ukl = zeros(1,length(t));

ykf = zeros(1,length(t)); % forward direction
ykl = zeros(1,length(t)); % lateral direction
CoMf = zeros(1,length(t));
CoMl = zeros(1,length(t));
ef = 0; % summation of error
el = 0;

prevInd=1:length(N);
prevEnd=(length(t)-length(N)); %End of previewable window
A=sys.A;
B=sys.B;
C=sys.C;
for k = 1:prevEnd
    %Store current state  in CoMf,l
    CoMf(k) = xkf(1,k);
    CoMl(k) = xkl(1,k);
    
    % C = output matrix, xk is the state vector at time index k
    ykf(k) = C*xkf(:,k); 
    ykl(k) = C*xkl(:,k);
    
    ef = ef + ykf(k) - xZMP(k);
    el = el + ykl(k) - yZMP(k);
    prevf = sum(Gp.*xZMP(k+prevInd));
    prevl = sum(Gp.*yZMP(k+prevInd));
    
    ukf(k) = -Ke*ef - Kx*xkf(:,k) - prevf;
    ukl(k) = -Ke*el - Kx*xkl(:,k) - prevl;
    xkf(:,k+1) = A*xkf(:,k)+B*ukf(k);
    xkl(:,k+1) = A*xkl(:,k)+B*ukl(k);
end
%Second loop to deal with tail end, where there is no preview trajectory
%Use last position to fill "dummy" ZMP

ykf(prevEnd:end) = ykf(prevEnd);  
ykl(prevEnd:end) = ykl(prevEnd);
CoMf(prevEnd:end) = xkf(1,prevEnd);
CoMl(prevEnd:end) = xkl(1,prevEnd);

for k= (length(t)-length(N)):length(t)
    ukf(k) = -Ke*ef - Kx*xkf(:,k) - prevf;
    ukl(k) = -Ke*el - Kx*xkl(:,k) - prevl;
    xkf(:,k+1) = A*xkf(:,k)+B*ukf(k);
    xkl(:,k+1) = A*xkl(:,k)+B*ukl(k);
end

if DEBUG
%     figure(1)
%     plot(N,Gp,'linewidth',3);
%     title('Preview Gain, Gp, vs. Time');
%     xlabel('Time (second)');
%     ylabel('Magnitude of Gp');
%     legend('Gp');
%     grid on
    
    figure(2)
    plot(t,xZMP,'b','linewidth',3);grid on
    hold on
    plot(t,ykf,'r-.','linewidth',4)
    hold on
    plot(t,CoMf,'g--','linewidth',3);
    title('Desired Forward ZMP, ZMP via Preview, and CoM via Preview');
    xlabel('Time (second)');
    ylabel('Distance in Forward Direction (m)');
    legend('ZMP ref.','ZMP Preview','CoM Preview');
    hold off
    
    figure(3)
    plot(t,yZMP,'b','linewidth',3);grid on
    hold on
    plot(t,ykl,'r-.','linewidth',4)
    hold on
    plot(t,CoMl,'g--','linewidth',3);
    title('Desired lateral ZMP, ZMP via Preview, and CoM via Preview');
    xlabel('Time (second)');
    ylabel('Distance in Lateral Direction (m)');
    legend('ZMP ref.','ZMP Preview','CoM Preview');
    hold off
    
%     figure(4)
%     plot(t,yZMP,'linewidth',3);
%     title('ZMP in Lateral');
%     xlabel('Time (second)');
%     ylabel('Step Distance');
%     ylim([-8 8])
%     legend('ZMP in Y');
%     grid on
%     hold off
end

end