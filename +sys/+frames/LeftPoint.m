function [contact, fric_coef, geometry] = LeftPoint(robot)
    
    param = sys.GetExtraParams();
       
    % r_foot
    l_knee_frame = robot.Joints(getJointIndices(robot, 'l_FB_1'));
	contact = CoordinateFrame(...
        'Name','LeftPoint',...
        'Reference',l_knee_frame,...
        'Offset',[0,0,-param.ka],...
        'R',[0,0,0]... % z-axis is the normal axis, so no rotation required
        );
%     robot.ContactPoints.LeftPoint = contact;
    
    fric_coef.mu = param.mu;
    fric_coef.gamma = param.gamma;

    geometry = [];
    
end