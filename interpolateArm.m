function [X,Y,Z]=interpolateArm(Times,Frames,dt)
    N=Times(end)-Times(1);
    
    %Intervals defined as closed over start and finish times, causing
    %recalculation at each end (ugly but acceptable)
    X=interpolateAxis(Times,Frames(1,:),dt);
    Y=interpolateAxis(Times,Frames(2,:),dt);
    Z=interpolateAxis(Times,Frames(3,:),dt);
end
function X=interpolateAxis(Times, Frames,dt)
    N=floor((Times(end)-Times(1))/dt)+1;
    X=zeros(1,N);
    X(1)=Frames(1);
    for k=2:length(Times)
        k
        deltaX=Frames(k)-Frames(k-1);
        deltaT=Times(k)-Times(k-1);
        dx=deltaX*dt/deltaT
        
        for t=Times(k-1):dt:Times(k)
            n=round((t-Times(1))/dt)+1;
            X(n+1)=X(n)+dx;
        end
    end
end
