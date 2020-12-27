function [contact, fric_coef, geometry] = RightPoint(robot)
    
    param = sys.GetExtraParams();
       
    % r_foot
    r_knee_frame = robot.Joints(getJointIndices(robot, 'r_FB_1'));
	contact = CoordinateFrame(...
        'Name','RightPoint',...
        'Reference',r_knee_frame,...
        'Offset',[0,0,-param.ka],...
        'R',[0,0,0]... % z-axis is the normal axis, so no rotation required
        );
%     robot.ContactPoints.RightPoint = contact;
    
    fric_coef.mu = param.mu;
    fric_coef.gamma = param.gamma;

    geometry = [];
    
end