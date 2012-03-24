function stepAnimateSolution(Angles,torsoInput,Frequency,IKSolver,h,walkarea)
            global DEBUG
            %TODO: Ensure this animation works after these tweaks
            if ~isempty(h)
                figure(h);
                axis(walkarea);
            else
                h=figure('Position',[50 50 640 480]);
            end
            [az,el]=view;
            clf
            cmap=colormap(copper(size(Angles,2)));
            %az=30;el=30;
            view([az,el])
            
            [h_leg,h_rot]=renderFrame(IKSolver,[0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]',[0,0,250,0,0,0]',[0,0,0]);
            
            frameSkip=ceil(Frequency/20);
            framerate=Frequency/frameSkip;
            frames=2:frameSkip:size(Angles,2);
            frametime=1/framerate;
            j=1;
            if ~DEBUG
                aviobj = avifile('turningmovie.avi','fps',framerate);
            end
            for k=frames
                tic
                if ~ishandle(h)
                    break;
                end
                renderFrame(IKSolver,Angles(:,k),torsoInput(:,k),cmap(k,:),h_leg,h_rot,walkarea);
                pause
                if DEBUG && toc<frametime
                    pause(frametime-toc);
                end
                drawnow();
                if ~DEBUG
                    frame = getframe(gca);
                    aviobj = addframe(aviobj,frame);
                end
            end
            if ~DEBUG
                aviobj = close(aviobj);
            end
end

function [h_leg,h_rot]=renderFrame(IKSolver,angles,parameters,color,h_leg,h_rot,walkarea)
            [leftLeg,rightLeg,leftArm,rightArm]=IKSolver.Points(angles,parameters);
            leftLeg=cumsum(leftLeg,2);
            rightLeg=cumsum(rightLeg,2);
            leftArm=cumsum(leftArm,2);
            rightArm=cumsum(rightArm,2);
            [R_LeftLeg,R_RightLeg,R_LeftArm,R_RightArm]=IKSolver.Csys(angles,parameters);
            % Initialize plot without handle
            
            if nargin<=4
                h_leg{1}=line(leftLeg(1,:),leftLeg(2,:),leftLeg(3,:),'Color',color,'LineWidth',3);
                h_leg{2}=line(rightLeg(1,:),rightLeg(2,:),rightLeg(3,:),'Color',color,'LineWidth',3);
                h_leg{3}=line(leftArm(1,:),leftArm(2,:),leftArm(3,:),'Color',color,'LineWidth',3);
                h_leg{4}=line(rightArm(1,:),rightArm(2,:),rightArm(3,:),'Color',color,'LineWidth',3);
                h_rot{1}=Csysplot(R_LeftLeg(:,1:3),leftLeg(:,1),[]);
                h_rot{2}=Csysplot(R_RightLeg(:,1:3),rightLeg(:,1),[]);
                h_rot{3}=Csysplot(R_LeftArm(:,1:3),R_LeftArm(:,1),[]);
                h_rot{4}=Csysplot(R_RightArm(:,1:3),R_RightArm(:,1),[]);
                for k=2:size(R_LeftLeg,2)/3
                    h_rot{1}(:,k)=Csysplot(R_LeftLeg(:,3*k-2:3*k),leftLeg(:,k),[]);
                    h_rot{2}(:,k)=Csysplot(R_RightLeg(:,3*k-2:3*k),rightLeg(:,k),[]);
                end
                for  k=2:size(R_LeftArm,2)/3
                    h_rot{3}(:,k)=Csysplot(R_LeftArm(:,3*k-2:3*k),leftArm(:,k),[]);
                    h_rot{4}(:,k)=Csysplot(R_RightArm(:,3*k-2:3*k),rightArm(:,k),[]);
                end
                title('Animation of MiniHubo leg with CSys')
                xlabel('X Axis')
                ylabel('Y Axis')
                zlabel('Z Axis')
                grid on
                axis equal
            else
                %update rotation matrices
                for k=1:size(R_LeftLeg,2)/3
                    Csysplot(R_LeftLeg(:,3*k-2:3*k),leftLeg(:,k),h_rot{1}(:,k));
                    Csysplot(R_RightLeg(:,3*k-2:3*k),rightLeg(:,k),h_rot{2}(:,k));
                end
                for k=1:size(R_LeftArm,2)/3
                    Csysplot(R_LeftArm(:,3*k-2:3*k),leftArm(:,k),h_rot{3}(:,k));
                    Csysplot(R_RightArm(:,3*k-2:3*k),rightArm(:,k),h_rot{4}(:,k));
                end
                %update leg plot
                set(h_leg{1},'Color',color);
                set(h_leg{1},'XData',leftLeg(1,:))
                set(h_leg{1},'YData',leftLeg(2,:))
                set(h_leg{1},'ZData',leftLeg(3,:))
                set(h_leg{2},'Color',color);
                set(h_leg{2},'XData',rightLeg(1,:))
                set(h_leg{2},'YData',rightLeg(2,:))
                set(h_leg{2},'ZData',rightLeg(3,:))
                set(h_leg{3},'Color',color);
                set(h_leg{3},'XData',leftArm(1,:))
                set(h_leg{3},'YData',leftArm(2,:))
                set(h_leg{3},'ZData',leftArm(3,:))
                set(h_leg{4},'Color',color);
                set(h_leg{4},'XData',rightArm(1,:))
                set(h_leg{4},'YData',rightArm(2,:))
                set(h_leg{4},'ZData',rightArm(3,:))
                axis(walkarea)
            end
            
        end
        
        function ht=Csysplot(rotmat,p0,ht)
            scale=20;
            width=2;
            DataX=cumsum([p0,rotmat(:,1)*scale],2);
            DataY=cumsum([p0,rotmat(:,2)*scale],2);
            DataZ=cumsum([p0,rotmat(:,3)*scale],2);
            if nargin<3 || isempty(ht)
                %make line thisects
                ht(1,1)=line(DataX(1,:),DataX(2,:),DataX(3,:),'Color','r','LineWidth',width);
                ht(2,1)=line(DataY(1,:),DataY(2,:),DataY(3,:),'Color','g','LineWidth',width);
                ht(3,1)=line(DataZ(1,:),DataZ(2,:),DataZ(3,:),'Color','b','LineWidth',width);
            else
                set(ht(1),'XData',DataX(1,:));
                set(ht(2),'XData',DataY(1,:));
                set(ht(3),'XData',DataZ(1,:));
                set(ht(1),'YData',DataX(2,:));
                set(ht(2),'YData',DataY(2,:));
                set(ht(3),'YData',DataZ(2,:));
                set(ht(1),'ZData',DataX(3,:));
                set(ht(2),'ZData',DataY(3,:));
                set(ht(3),'ZData',DataZ(3,:));
            end
        end