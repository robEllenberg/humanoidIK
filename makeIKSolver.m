function IKSolver=makeIKSolver(skeletonData,solverPrefix,mexFlag)
    % IK function generator for humanoid:
    %
    % This function uses a matlab data file containing 2 arrays
    %
    % leftLegSkeleton is a 3x7 matrix of vectors to each successive leg joint, in
    % the home position
    %
    % leftArmSkeleton is a 3x5 matrix of similar vectors for the arm
    %
    % solverPrefix is an optional string containing a name prefix for the
    % output functions and solver struct
    %
    % the mexFlag input variable is a 2 element array which controls MEX and C
    % generation:
    % mexFlag(1) == 1 --> make MEX functions for each IK function
    % mexFlag(2) == 1 --> produce C versions of all functions without compiling
    %
    % Note that with static function names, the IKSolver structure is not useful anymore
    %

    % Define solver function names for given prefix
    if nargin <2
        solverPrefix='default_';
        mexFlag=[0,0];
    end
    J=sprintf('%sJacobian.m',solverPrefix);
    FK=sprintf('%sForwardKinematics.m',solverPrefix);
    FK2=sprintf('%sConstraintEquations.m',solverPrefix);
    PP=sprintf('%sPlotPoints.m',solverPrefix);
    RMat=sprintf('%sRotationMatrices.m',solverPrefix);
    IK=sprintf('%sIKSolve.m',solverPrefix);
    
    load(skeletonData,'leftLegSkeleton','leftArmSkeleton','footWidth','footLength')
    
    %% Left Leg Definitions
    syms HY LHY LHR LHP LKP LAP LAR real;
    %syms TSOX TSOY TSOZ tPitch tRoll tYaw real;
    % Rotation matrices for each joint
    R_TSO=eye(3);
    R_HY=R_TSO*Rz(HY);
    R_LHY=R_HY*Rz(LHY);
    R_LHR=R_LHY*Rx(LHR);
    R_LHP=R_LHR*Ry(-LHP);
    R_LKP=R_LHP*Ry(LKP);
    R_LAP=R_LKP*Ry(-LAP);
    R_LAR=R_LAP*Rx(LAR);
    
    % Link offsets between axes
    %d_TSO=[TSOX;TSOY;TSOZ];
    d_TSO=[0;0;0];
    d_TSO_HY=leftLegSkeleton(:,1);
    d_HY_LHY=leftLegSkeleton(:,2);
    d_LHY_LHR=leftLegSkeleton(:,3);
    d_LHR_LHP=leftLegSkeleton(:,4);
    d_LHP_LKP=leftLegSkeleton(:,5);
    d_LKP_LAP=leftLegSkeleton(:,6);
    d_LAP_LAR=leftLegSkeleton(:,7);
    d_LAR_Foot=leftLegSkeleton(:,8);
    
    % Vector from hip to foot
    P_Torso_LeftFoot=d_TSO+R_TSO*d_TSO_HY+...
        R_HY*d_HY_LHY+...
        R_LHY*d_LHY_LHR+...
        R_LHR*d_LHR_LHP+...
        R_LHP*d_LHP_LKP+...
        R_LKP*d_LKP_LAP+...
        R_LAP*d_LAP_LAR+...
        R_LAR*d_LAR_Foot;
    
    %% Right foot definitions
    syms RHY RHR RHP RKP RAP RAR real;
    % Rotation matrices for each joint
    
    R_RHY=(Rz(RHY)'*R_HY')';
    R_RHR=(Rx(-RHR)'*R_RHY')';
    R_RHP=(Ry(-RHP)'*R_RHR')';
    R_RKP=(Ry(RKP)'*R_RHP')';
    R_RAP=(Ry(-RAP)'*R_RKP')';
    R_RAR=(Rx(RAR)'*R_RAP')';
    
    % Link offsets between axes
    LtoRTransform=diag([1,-1,1]);
    d_HY_RHY  = LtoRTransform * d_HY_LHY;
    d_RHY_RHR = LtoRTransform * d_LHY_LHR;
    d_RHR_RHP = LtoRTransform * d_LHR_LHP;
    d_RHP_RKP = LtoRTransform * d_LHP_LKP;
    d_RKP_RAP = LtoRTransform * d_LKP_LAP;
    d_RAP_RAR = LtoRTransform * d_LAP_LAR;
    d_RAR_Foot= LtoRTransform * d_LAR_Foot;
    
    % Vector from hip to foot
    P_Torso_RightFoot=d_TSO+R_TSO*d_TSO_HY+...
        R_HY*d_HY_RHY+...
        R_RHY*d_RHY_RHR+...
        R_RHR*d_RHR_RHP+...
        R_RHP*d_RHP_RKP+...
        R_RKP*d_RKP_RAP+...
        R_RAP*d_RAP_RAR+...
        R_RAR*d_RAR_Foot;
    
    %% Left Arm
    syms LSP LSR LSY LEB real
    R_LSP=Ry(LSP);
    R_LSR=(Rx(LSR)'*(R_LSP)')';
    R_LSY=(Rz(LSY)'*(R_LSR)')';
    R_LEB=(Ry(-LEB)'*(R_LSY)')';
    
    d_TSO_LSP=leftArmSkeleton(:,1);
    d_LSP_LSR=leftArmSkeleton(:,2);
    d_LSR_LSY=leftArmSkeleton(:,3);
    d_LSY_LEB=leftArmSkeleton(:,4);
    d_LEB_LWR=leftArmSkeleton(:,5);
    
    P_Torso_LeftWrist=d_TSO+R_TSO*d_TSO_LSP+...
        R_LSP*d_LSP_LSR+...
        R_LSR*d_LSR_LSY+...
        R_LSY*d_LSY_LEB+...
        R_LEB*d_LEB_LWR;
    
    %% Right Arm
    syms RSP RSR RSY REB real
    R_RSP=Ry(RSP);
    R_RSR=(Rx(-RSR)'*(R_RSP)')';
    R_RSY=(Rz(-RSY)'*(R_RSR)')';
    R_REB=(Ry(-REB)'*(R_RSY)')';
    
    d_TSO_RSP=LtoRTransform*d_TSO_LSP;
    d_RSP_RSR=LtoRTransform*d_LSP_LSR;
    d_RSR_RSY=LtoRTransform*d_LSR_LSY;
    d_RSY_REB=LtoRTransform*d_LSY_LEB;
    d_REB_RWR=LtoRTransform*d_LEB_LWR;
    
    P_Torso_RightWrist=d_TSO+R_TSO*d_TSO_RSP+...
        R_RSP*d_RSP_RSR+...
        R_RSR*d_RSR_RSY+...
        R_RSY*d_RSY_REB+...
        R_REB*d_REB_RWR;
    
    %% Additional constraints - used to define foot orientation
    % 1) sine of angle from foot X axis to plane (Z component of foot X vector)
    % 2) sine of angle from foot Y axis to plane (Z component of foot Y vector)
    % 3) sine of foot yaw angle (Y component of foot X vector)
    leftFootXVector=R_LAR*[1;0;0];
    leftFootYVector=R_LAR*[0;1;0];
    
    rightFootXVector=R_RAR*[1;0;0];
    rightFootYVector=R_RAR*[0;1;0];
    
    %NOTE the order change here!! this is to simplify what we have to deal with
    %and ignore foot angles entirely.
    
    forwardKinematics=[P_Torso_LeftFoot;
        P_Torso_RightFoot;
        P_Torso_LeftWrist;
        P_Torso_RightWrist];
    
    constraintSet=[forwardKinematics;
        leftFootXVector(3);
        leftFootYVector(3);
        leftFootXVector(2);
        rightFootXVector(3);
        rightFootYVector(3);
        rightFootXVector(2)];
    
    % Vector of joint angles (symbolic)
    vars=[HY;
        LHY;
        LHR;
        LHP;
        LKP;
        LAP;
        LAR;
        RHY;
        RHR;
        RHP;
        RKP;
        RAP;
        RAR;
        LSP;
        LSR;
        LSY;
        LEB;
        RSP;
        RSR;
        RSY;
        REB];
    
    %Define jacobian function and constraints
    Jsym=jacobian(constraintSet,vars);
    IKSolver.Jacobian=matlabFunction(Jsym,'vars',{vars},'file',J);
    IKSolver.forwardKinematics=matlabFunction(forwardKinematics,'vars',{vars},'file',FK);
    IKSolver.constraintEquations=matlabFunction(constraintSet,'vars',{vars},'file',FK2);
    
    %% Graphics functions
    
    d_Foot=[footLength/2,0,-footLength,0,footLength;
        0,-footWidth/2,0,footWidth,0;
        0,0,0,0,0];
    plotPoints1=[d_TSO+R_TSO*d_TSO_HY,...
        R_HY*d_HY_LHY,R_LHY*d_LHY_LHR,...
        R_LHR*d_LHR_LHP,R_LHP*d_LHP_LKP,...
        R_LKP*d_LKP_LAP,R_LAP*d_LAP_LAR,...
        R_LAR*d_LAR_Foot,R_LAR*d_Foot];
    plotPoints2=[d_TSO+R_TSO*d_TSO_HY,...
        R_HY*d_HY_RHY,R_RHY*d_RHY_RHR,...
        R_RHR*d_RHR_RHP,R_RHP*d_RHP_RKP,...
        R_RKP*d_RKP_RAP,R_RAP*d_RAP_RAR,...
        R_RAR*d_RAR_Foot,R_RAR*d_Foot];
    plotPoints3=[d_TSO,R_TSO*d_TSO_LSP,R_LSP*d_LSP_LSR,...
        R_LSR*d_LSR_LSY,R_LSY*d_LSY_LEB,R_LEB*d_LEB_LWR];
    plotPoints4=[d_TSO,R_TSO*d_TSO_RSP,R_RSP*d_RSP_RSR,...
        R_RSR*d_RSR_RSY,R_RSY*d_RSY_REB,R_REB*d_REB_RWR];
    
    IKSolver.Points=matlabFunction(plotPoints1,...
        plotPoints2,...
        plotPoints3,...
        plotPoints4,'vars',{vars},'file',PP);
    
    rotationMatrices1=[R_HY,R_LHY,R_LHR,R_LHP,R_LKP,R_LAP,R_LAR];
    rotationMatrices2=[R_HY,R_RHY,R_RHR,R_RHP,R_RKP,R_RAP,R_RAR];
    rotationMatrices3=[R_TSO,R_LSP,R_LSR,R_LSY,R_LEB];
    rotationMatrices4=[R_TSO,R_RSP,R_RSR,R_RSY,R_REB];
    
    IKSolver.Csys=matlabFunction(rotationMatrices1,...
        rotationMatrices2,...
        rotationMatrices3,...
        rotationMatrices4,'vars',{vars},'file',RMat);
    
    %% Other misc data
    IKSolver.method='';
    IKSolver.vars=vars;
    %IKSolver.tolerances=ones(IKSolver.size(1),1)*.01;
    
    %% Embedded Matlab Conversion
    %Dummy variables to produce approproate sizes of input
    q=zeros(size(vars));
    p=zeros(12,1);
    if mexFlag(1)
        %p0=zeros(size(parameters));
        eval(sprintf('codegen -args {q} -config:mex %s',J));
        eval(sprintf('codegen -args {q} -config:mex %s',FK));
        eval(sprintf('codegen -args {q} -config:mex %s',PP));
        eval(sprintf('codegen -args {q} -config:mex %s',RMat));
        eval(sprintf('codegen -args {p q} -config:mex %s',IK));
    end
    if mexFlag(2)
        eval(sprintf('codegen -args {q} -c %s',J));
        eval(sprintf('codegen -args {q} -c %s',FK));
        eval(sprintf('codegen -args {q} -c %s',PP));
        eval(sprintf('codegen -args {q} -c %s',RMat));
        eval(sprintf('codegen -args {p q} -c %s',IK));
        %TODO: Convert this to the ccode function to avoid stupid errors
        
    end
    solverName=sprintf('%sIKSolver',solverPrefix);
    save(solverName,'IKSolver');
end

function R=Rx(t)
    R=[1 0 0;0 cos(t) -sin(t);0 sin(t) cos(t)];
end
function R=Ry(t)
    R=[cos(t) 0 sin(t);0 1 0;-sin(t) 0 cos(t)];
end
function R=Rz(t)
    R=[cos(t) -sin(t) 0;sin(t) cos(t) 0;0 0 1];
end
