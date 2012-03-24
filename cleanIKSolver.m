%Clean up IK solver functions for a given name
function IKSolver=cleanIKSolver(solverPrefix,mexFlag)
    
    J=sprintf('%sJacobian.m',solverPrefix);
    FK=sprintf('%sForwardKinematics.m',solverPrefix);
    FK2=sprintf('%sConstraintEquations.m',solverPrefix);
    PP=sprintf('%sPlotPoints.m',solverPrefix);
    RMat=sprintf('%sRotationMatrices.m',solverPrefix);
    IK=sprintf('%sIKSolve.m',solverPrefix);
    solverName=sprintf('%sIKSolver',solverPrefix);
    
    delete(J,FK,FK2,PP,RMat,IK,[solverName '.mat']);
    if mexFlag(1)
       %TODO: Cleanup codegen folder
    end
    if mexFlag(2)      
        %TODO: Cleanup c files in codegen folder
    end
    
    
end