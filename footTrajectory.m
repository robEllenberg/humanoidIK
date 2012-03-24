function [xL,yL,zL,xR,yR,zR,thetaL,thetaR] =  footTrajectory(leftPath, rightPath,thetaLeft,thetaRight,robot)
    % minihubo foot trajectory for planar walking
    % t is input time (not vector yet)
    
    X=1;
    Y=2;
    pds=robot.doubleSupportRatio;
    dt=robot.dt;
    zAmp=robot.stepHeight;
    tStep=floor(robot.tStep/dt)*dt;
    tds=floor(pds*tStep/dt)*dt;
    % Use sanitized values to produce time offset vectors for ss and ds
    dsTVec=dt:dt:tds;
    ssTVec=(tds+dt):dt:tStep;
    Tss=tStep-tds;
        
    lds=length(dsTVec);
    lss=length(ssTVec);
        
    % Copy format of ZMP --> Hip
    % 1) initial pose standing still
    T=0:dt:(3*tStep);
    xL=ones(size(T)).*leftPath(X,1);
    yL=ones(size(T)).*leftPath(Y,1);
    zL=zeros(size(T));
    xR=ones(size(T)).*rightPath(X,1);
    yR=ones(size(T)).*rightPath(Y,1);
    zR=zeros(size(T));
    
    thetaL=ones(size(T)).*thetaLeft(1);
    thetaR=ones(size(T)).*thetaRight(1);
    %loop through and make 1 cycle at a time of ZMP
    %TODO: figure out the calculation to find total length of a vector,
    %use a for loop to build indices, and store values to given indices.
    %VERY IMPORTANT for memory usage, since we only have 1 gig available
    for k=2:length(leftPath)
                
        % DSS phase, hold position
        xL=[xL,ones(1,lds)*leftPath(X,k-1)];
        yL=[yL,ones(1,lds)*leftPath(Y,k-1)];
        zL=[zL,zeros(1,lds)];%eventually generalize to make stepping trajectory
        
        thetaL=[thetaL,ones(1,lds)*thetaLeft(k-1)];
        
        % SS phase, liftoff and move
        xL=[xL,cycloidInterp(Tss,leftPath(X,k-1),leftPath(X,k),dt)];
        yL=[yL,cycloidInterp(Tss,leftPath(Y,k-1),leftPath(Y,k),dt)];
        zL=[zL,sinfoot(zAmp,Tss,dt)];
        thetaL=[thetaL,cycloidInterp(Tss,thetaLeft(k-1),thetaLeft(k),dt)];
        
        % Support foot phase, land and hold
        xL=[xL,ones(1,lds+lss)*leftPath(X,k)];
        yL=[yL,ones(1,lds+lss)*leftPath(Y,k)];
        zL=[zL,zeros(1,lss+lds)];
        thetaL=[thetaL,ones(1,lss+lds)*thetaLeft(k)];
        
        % Extended Support/DS phase
        xR=[xR,ones(1,2*lds+lss)*rightPath(X,k-1)];
        yR=[yR,ones(1,2*lds+lss)*rightPath(Y,k-1)];
        zR=[zR,zeros(1,2*lds+lss)];
        thetaR=[thetaR,ones(1,2*lds+lss)*thetaRight(k-1)];
        % SS phase, liftoff and move
        xR=[xR,cycloidInterp(Tss,rightPath(X,k-1),rightPath(X,k),dt)];
        yR=[yR,cycloidInterp(Tss,rightPath(Y,k-1),rightPath(Y,k),dt)];
        zR=[zR,sinfoot(zAmp,Tss,dt)];
        thetaR=[thetaR,cycloidInterp(Tss,thetaRight(k-1),thetaRight(k),dt)];
        
    end
    n=round(3*tStep/dt);
    xL=[xL,ones(1,n).*leftPath(X,end)];
    yL=[yL,ones(1,n).*leftPath(Y,end)];
    zL=[zL,zeros(1,n)];
    thetaL=[thetaL,ones(1,n).*thetaLeft(end)];
    
    xR=[xR,ones(1,n).*rightPath(X,end)];
    yR=[yR,ones(1,n).*rightPath(Y,end)];
    zR=[zR,zeros(1,n)];
    thetaR=[thetaR,ones(1,n).*thetaRight(end)];
end

function Yi=cycloidInterp(T,Y0,Y1,dt)
    %Always start at dt so that the offset time can be neatly added
    t=dt:dt:T;
    w=2*pi/T;
    Yi=cycloid(t,w)*(Y1-Y0)+Y0;
end

function Z=sinfoot(ZAmp,Tss,dt)
    %Always start at dt so that the offset time can be neatly added
    t=dt:dt:Tss;
    w=2*pi/Tss;
    Z=ZAmp*sin(w/2*t);
end

function y=cycloid(t,omega)
%vectorized cycloid function
if nargin<2
    y=1/(2*pi)*(2*pi*t-sin(2*pi*t));
else
    y=1/(2*pi)*(omega*(t)-sin(omega*t));
end
end

function y=interruptedCycloid(t, Tp, Tss, amp, omega)
    cyc_max = amp;
    if (mod(t,Tp) <= Tss)
        y = cyc_max*( cycloid(mod(t,Tp), omega)+ floor(t/Tp) );
    else
        y = cyc_max * ceil(t/Tp);
    end
end

function [x,y,z]=foot(t, tStep, pds, Xamp, Yamp, Zamp, Xoff, Yoff, Zoff)
   Tss=tStep*(1-pds);
    Tp=2*tStep;
    w=2*pi/Tss;
    
    x = interruptedCycloid(t, Tp, Tss, Xamp, w) + Xoff;
    y = interruptedCycloid(t, Tp, Tss, Yamp, w) + Yoff;
    
    if (mod(t, Tp) <= Tss )
        z = Zamp*sin(w/2 * ( mod(t,(Tp/2)) ) )+Zoff;
    else
        z = Zoff;
    end
end