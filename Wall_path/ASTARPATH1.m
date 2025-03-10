function OptimalPath=ASTARPATH(StartX,StartY,MAP,GoalRegister,Connecting_Distance)
%Version 1.0
% By Einar Ueland 2nd of May, 2016

%FINDING ASTAR PATH IN AN OCCUPANCY GRID


%nNeighboor=3;
% Preallocation of Matrices
[Height,Width]=size(MAP); %Height and width of matrix
GScore=zeros(Height,Width);           %Matrix keeping track of G-scores 
FScore=single(inf(Height,Width));     %Matrix keeping track of F-scores (only open list) 
Hn=single(zeros(Height,Width));       %Heuristic matrix
OpenMAT=int8(zeros(Height,Width));    %Matrix keeping of open grid cells
ClosedMAT=int8(zeros(Height,Width));  %Matrix keeping track of closed grid cells
ClosedMAT(MAP==1)=1;                  %Adding object-cells to closed matrix
ParentX=int16(zeros(Height,Width));   %Matrix keeping track of X position of parent
ParentY=int16(zeros(Height,Width));   %Matrix keeping track of Y position of parent


%%% Setting up matrices representing neighboors to be investigated
NeighboorCheck=ones(2*Connecting_Distance+1);
Dummy=2*Connecting_Distance+2;
Mid=Connecting_Distance+1;
for i=1:Connecting_Distance-1
NeighboorCheck(i,i)=0;
NeighboorCheck(Dummy-i,i)=0;
NeighboorCheck(i,Dummy-i)=0;
NeighboorCheck(Dummy-i,Dummy-i)=0;
NeighboorCheck(Mid,i)=0;
NeighboorCheck(Mid,Dummy-i)=0;
NeighboorCheck(i,Mid)=0;
NeighboorCheck(Dummy-i,Mid)=0;
end
NeighboorCheck(Mid,Mid)=0;

[row, col]=find(NeighboorCheck==1);
Neighboors=[row col]-(Connecting_Distance+1);
N_Neighboors=size(col,1);
%%% End of setting up matrices representing neighboors to be investigated


%%%%%%%%% Creating Heuristic-matrix based on distance to nearest  goal node
[col, row]=find(GoalRegister==1);
RegisteredGoals=[row col];
Nodesfound=size(RegisteredGoals,1);

for k=1:size(GoalRegister,1)
    for j=1:size(GoalRegister,2)
        if MAP(k,j)==0
            Mat=RegisteredGoals-(repmat([j k],(Nodesfound),1));
            Distance=(min(sqrt(sum(abs(Mat).^2,2))));
            Hn(k,j)=Distance;
        end
    end
end
%End of creating Heuristic-matrix. 

%Note: If Hn values is set to zero the method will reduce to the Dijkstras method.

%Initializign start node with FValue and opening first node.
FScore(StartX,StartY)=Hn(StartX,StartY);         
OpenMAT(StartX,StartY)=1;   




while 1==1 %Code will break when path found or when no path exist
    MINopenFSCORE=min(min(FScore));
    if MINopenFSCORE==inf;
    %Failuere!
    OptimalPath=[inf];
    RECONSTRUCTPATH=0;
     break
    end
    [CurrentX,CurrentY]=find(FScore==MINopenFSCORE);
    CurrentY=CurrentY(1);
    CurrentX=CurrentX(1);

    if GoalRegister(CurrentX,CurrentY)==1
    %GOAL!!
        RECONSTRUCTPATH=1;
        break
    end
    
  %Remobing node from OpenList to ClosedList  
    OpenMAT(CurrentX,CurrentY)=0;
    FScore(CurrentX,CurrentY)=inf;
    ClosedMAT(CurrentX,CurrentY)=1;
    for p=1:N_Neighboors
        i=Neighboors(p,1); %X
        j=Neighboors(p,2); %Y
        if CurrentX+i<1||CurrentX+i>Height||CurrentY+j<1||CurrentY+j>Width
            continue
        end
        Flag=1;
        if(ClosedMAT(CurrentX+i,CurrentY+j)==0) %Neiboor is open;
            if (abs(i)>1||abs(j)>1);   
                % Need to check that the path does not pass an object
                JumpCells=2*max(abs(i),abs(j))-1;
                for K=1:JumpCells;
                    XPOS=round(K*i/JumpCells);
                    YPOS=round(K*j/JumpCells);
            
                    if (MAP(CurrentX+XPOS,CurrentY+YPOS)==1)
                        Flag=0;
                    end
                end
            end
             %End of  checking that the path does not pass an object

            if Flag==1;           
                tentative_gScore = GScore(CurrentX,CurrentY) + sqrt(i^2+j^2);
                if OpenMAT(CurrentX+i,CurrentY+j)==0
                    OpenMAT(CurrentX+i,CurrentY+j)=1;                    
                elseif tentative_gScore >= GScore(CurrentX+i,CurrentY+j)
                    continue
                end
                ParentX(CurrentX+i,CurrentY+j)=CurrentX;
                ParentY(CurrentX+i,CurrentY+j)=CurrentY;
                GScore(CurrentX+i,CurrentY+j)=tentative_gScore;
                FScore(CurrentX+i,CurrentY+j)= tentative_gScore+Hn(CurrentX+i,CurrentY+j);
            end
        end
    end
end

k=2;
if RECONSTRUCTPATH
    OptimalPath(1,:)=[CurrentX CurrentY];
    while RECONSTRUCTPATH
        CurrentXDummy=ParentX(CurrentX,CurrentY);
        CurrentY=ParentY(CurrentX,CurrentY);
        CurrentX=CurrentXDummy;
        OptimalPath(k,:)=[CurrentX CurrentY];
        k=k+1;
        if (((CurrentX== StartX)) &&(CurrentY==StartY))
            break
        end
    end
end


end

      
    
%{
line 93 -106 is using brute force to check if the path passes an object. For large connecting_distances
this uses alot of CPU time. If you have any more efficient way to add this effect in the code, pleas contact me and 
I will edit the code.  
%}
