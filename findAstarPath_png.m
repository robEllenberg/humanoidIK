function [ shortestPathGlobalX,shortestPathGlobalY ] = findAstarPath_ana35(startmm,goalmm)
%% INPUT START AND GOAL POSITIONS IN MILLIMETERS FROM THE TOP LEFT CORNER
%% OF MAP PNG WHERE X IS POSITIVE RIGHT AND Y IS POSITIVE DOWN. FUCTION
%% OUTPUTS A PATH WITH START POSITION AS ZERO AND PATH IS IN ROBOT
%% COORDINATES WHERE X IS POSITIVE RIGHT AND Y IS POSITIVE UP (IN THE PNG)

%% input example
%      [ shortestPathGlobalX,shortestPathGlobalY ] = findAstarPath_png([304.8,304.8],[2100.8,600])


%findAstarPath_ana35 is an A* search algorithm. This algorithm will search
%through an occupancy matrix and find the "best" route from a start to goal
%position based on criteria set in heuristic function.

%% NOTES:
% MAP SHOULD BE INPUT LIKE SO:
%    findAstarPath_ana35('OccupancyMatrix.txt')
% START AND GOAL ARE INPUT USING GINPUT.

%% Init, read in map

global keepgoing

%%

clc
clf
close all


% Initialize figure
h=figure(1);
set(h,'CloseRequestFcn',@closeme)

keepgoing=1;

figure(1)
 OccupancyInv = imread('obstacle_field_thresholded_low.png');
 OccupancyInv = OccupancyInv(:,:,1);
 OccupancyInv = double(OccupancyInv)/255;
 OccupancyInv = ceil(OccupancyInv);
 % The image is read in a massaged. Only one color channel is used as it
 % will be represented in "grayscale"
 
 image(255*OccupancyInv); colormap gray
 

 %%
   sizeMap = size(OccupancyInv);
   OccupancyInv   = [             zeros(1,sizeMap(2)+2)               ;  % pad the matrix with ones (barriers all around to avoid exceeding matrix dimensions
                         zeros(sizeMap(1),1) , OccupancyInv , zeros(sizeMap(1),1) ;
                                  zeros(1,sizeMap(2)+2)               ];
%%
map = 1-OccupancyInv;
imshow(OccupancyInv)
hold on


%%

    pixelsTOmm = 24.384; %mm/px

start = round([startmm(1)*(1/pixelsTOmm)   ,  startmm(2)*(1/pixelsTOmm)])         %round(ginput(1))
goal =  round([goalmm(1)*(1/pixelsTOmm)   , goalmm(2)*(1/pixelsTOmm)])              %round(ginput(1))



plot(start(1),start(2),'gs')
plot(goal(1),goal(2),'rs')

%pause(20)
%init open and closed lists
open =      [ start [0,0] 0 0 0 ]; %openCells [ [position] [parent] f g h ]
closed =    [ 0 0 0 0 0 0 0 ];

%init heuristic variables
g = 1;
h = 1;
f = 1;

%heuristic weights
gWeight = 1;
hWeight = .4;

%init boolean check for late loop
onOpen = 0;

%init x (current pos) at 0,0
x=[ 0 0 ];

%pause(2)

%map of zeros to be set to ones when point is added to list
mapped=zeros(size(map));

% fprintf('\nBe Patient :)\nI''m Calculating!\n\n')
%% while
while ((~isempty(open))&&(sum(x(1:2)~=goal))&&keepgoing)
    
    % pop x pos (initially the start pos) off of the open stack (open stack
    % used to keep track of points that should be looked at. Sorted so that
    % f(x) is smallest at the top and the first choice to be put on the
    % closed list.
    [x, open] = popOffStack(open);   %takes off top element in stack, returns [fist element , shorter stack]
    % check off with a 1 on the map
    mapped(x(2),x(1))=1;
    [X1,Y1]=find(mapped(open(:,2),open(:,1)==1));
    
    %add point taken from open stack onto the closed stack iff it is not
    %already on the list. % inList Function courtesy of Sean Mason.
    if ~inList(x(1:2),closed)
        closed = addToStack(x, closed);
    end
    %plot closed points
 %   plot(x(1),-x(2),'rs','markersize',4)
    
    %if the current position is the goal position, display goal and break
    %out of the while loop.
    if x(1:2) == goal
        display('GOAL!')
        break;
    end
    
    %iterate through 8 possible directions around x
    for i = -1:1
        for j = -1:1
        
        %rotation matrix*move vector rounded to nearest integer then flipped over x
        %axis
        move = [i, -j];
        %         pause(.01)
        %drawnow
        
        %compute neighbors of x by moving x in each of 8 directions
        neighbor = x(1:2)+move;
        
        %if the neighbor is not an obstacle and its not already on the
        %opened list
        if (map(neighbor(2),neighbor(1)) ~= 1) && mapped(neighbor(2),neighbor(1))==0
            %compute h(x), the euclidean distance from the neighbor to the
            %goal
            hypotenuse = norm(goal-(x(1:2)+move));
            gHyp = norm(neighbor-x(1:2));
            
            % if neighbor is on a diagonal, the g(x) distance is root2
            % while if its to the left, right, up, down, the g(x) is only 1
            g = gWeight*(x(6)+gHyp);
            
            %multiply by weights, compute f(x)
            h = hWeight*(hypotenuse);
            f =  g + h;
            
            %compute line to be added to open lsit and later to closed
            %list. line includes [pos, parent pos, f(x), g(x), h(x)]
            newPoint = [ x(1:2)+move , x(1:2) , f , g , h ];   % newPoint = [ newPointXY parentXY f(x) g(x) h(x) ]
            
            % for the entire list 'open'
            %for row = 1:size(open,1) % 1 to number of rows
            [found,indexMatch] = inList((x(1:2)+move) ,open);
                if found
                    %if x is already on the open list, onOpen is set to 1
                    %so that it is not added in the next loop.
                    onOpen = 1;
                    
                    if g < open(indexMatch,6)
                        % if x matches and x already on the open list and and its g(x)
                        % value is less that og the one on the list, the
                        % line on the open list is replaced with the
                        % recently constructed new line for x.
                        open(indexMatch,:) = newPoint;
                    else
                        break
                    end
                end
           % end
            
            %if x does not match another x on the open list, its newpoint
            %line is added to the open list.
            if onOpen == 0
                open = addToStack(newPoint, open);
                %open list points are plotted.
             %   plot(x(1)+move(1),-(x(2)+move(2)),'gs','markersize',6)
                
                
            end
            
            
        end
        
        onOpen = 0;
        
        end
    end
    
    %sorts list open by f(x) column in ascending order (smallest f(x) on
    %top)
    open = sortrows(open,5); 
end

%rest of code is run when while loop is broken out of. Exit criteria for
%main while loop:  break when open list is empty or when current position equals the goal.

%if the loop was broken out of because the open list was empty, this means
%that all options have been exhausted and there is no path to the goal
%(else statement displays this case to the user)

%if the list is not empty, then the loop was broken because the current
%position equalled the goal position. In this case, a loop runs through the
%closed list to recreate the shortest path. This is acheived by, starting
%with the goal's line, finding the the line of the previous point's parent.
%This line is added to the list and the closed list is run through again to
%find the most recent point's parent's line.
if ~isempty(open)
    
    shortestPath = closed(1,1:4);
    sizeClosed = size(closed);
    fillShorty = 1;
    
    for k = 2:(sizeClosed(1)-1)
        %fillShorty indexes to make sure, nomatter what line the parent's
        %line is found on, the shortestpath list is filled successivly.
        if closed(k,1:2)==shortestPath(fillShorty,3:4)
            shortestPath(fillShorty+1,:) = closed(k,1:4);
            fillShorty = fillShorty+1;
        end
    end
    
    figure(1)
    %plot the shortest path with a solid blue line.
    plot(shortestPath(:,1),shortestPath(:,2), 'c-', 'LineWidth',2)
    
    sizeShortestPath=size(shortestPath);
    %print the shortest path points
    %fprintf('\nShortest Path:\n')
    dex=1;
    shortestPathGlobalX = [];
    shortestPathGlobalY = [];
    
    for s=sizeShortestPath(1):-1:1
        %fprintf('   %g\t%g\n', shortestPath(s,1),shortestPath(s,2))
        
        xNormScale =  (shortestPath(s,1)-start(1))*pixelsTOmm;
        yNormScale = -(shortestPath(s,2)-start(2))*pixelsTOmm;
        
        shortestPathGlobalX(dex,:) = xNormScale;
        shortestPathGlobalY(dex,:) = yNormScale;
        
        dex = dex+1;
        
    end
else
    display('No Path Can Be Found!')
end



end

%%
%%%%%%%%% SUPPORTING FUNCTIONS %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%supporting fn, breaks out of code when map window is closed.
function closeme(~,~)
global keepgoing
keepgoing=0;
delete(gcf)
end

        function [found,indexMatch] = inList(element,matrix)
found= 0 ;
index = [];
indexMatch = 0;
[m,n]=size(matrix);

%HELPER TO CHECK IF ELEMENT IS IN A LIST
if isempty(matrix)==0
    
    c1 = matrix(:,1);
    c2 = matrix(:,2);
    index = find(c1 == element(1));
    
    for i = 1:length(index)
        
        if c2(index(i)) == element(2);
            found = 1;
            indexMatch = index(i);
        end
    end
end

end

% helper function to remove top element of a stack
function [outElement, newStack] = popOffStack(originalStack)
    [n,m] = size(originalStack);
    outElement = originalStack(1,:);
    newStack = originalStack(2:n,:);
end

% adds newElement (1xM row vector) to the originalStack (N x M array)
function outStack = addToStack(newElement, originalStack)
    outStack = [newElement; originalStack];
end
