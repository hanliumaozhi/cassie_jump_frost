 % Left Stance Domain for 2D
 %
 % Contact: Left Point
function domain = LeftStance(model, load_path)
    % construct the right stance domain of Art3-2D
    % 
    % Parameters:
    % model: the right body model of Art3 robot in 2D
    
    %% first make a copy of the robot model
    %	@note Do not directly assign with the model variable, since it is a
    %   handle object.
    domain = copy(model);
    % set the name of the new copy
    domain.setName('LeftStance');
    
    
    if nargin < 2
        load_path = [];
    end
    
    % Extract state variables
    x = domain.States.x;
    dx = domain.States.dx;
    
    %% Add contact
    % left foot point contact
    [left_foot, fric_coef,geometry] = sys.frames.LeftPoint(model);
    % convert a coordinate frame into a contact frame
    left_foot_contact = ToContactFrame(left_foot,...
        'PointContactWithFriction');
    % add contact
    domain = addContact(domain,left_foot_contact,fric_coef,geometry,load_path);
    
    
    
    %% Add event
    % height of non-stance foot (right foot)
    [right_foot] = sys.frames.RightPoint(model);
    p_swingFoot = getCartesianPosition(domain, right_foot);
    h_nsf = UnilateralConstraint(domain,p_swingFoot(3),'rightFootHeight','x');
    domain = addEvent(domain, h_nsf);
   
    %% Add Virtual Constraints
    
    % phase variable: linearized hip position
    %     l_hip_link = domain.Links(getLinkIndices(domain, 'r_uleg'));
    %     l_hip_frame = l_hip_link.Reference;
    %     l_hip_frame = domain.Joints(getJointIndices(domain,'l_leg_hpy'));
    %     l_ankle_frame = domain.Joints(getJointIndices(domain,'l_leg_aky'));
    %     p_lhp = getCartesianPosition(domain, l_hip_frame);
    %     p_lak = getCartesianPosition(domain, l_ankle_frame);
    %     p_hip = p_lhp(1) - p_lak(1);
    %     deltaPhip = linearize(p_hip,x);
    
    % @state based phase variable
    %     p = SymVariable('p',[2,1]);
    %     tau = (deltaPhip-p(2))/(p(1)-p(2));
    
    % @time based phase variable
    t = SymVariable('t');
    p = SymVariable('p',[2,1]);
    tau = (t-p(1))/(p(2)-p(1));
    
    
    % relative degree two outputs:
    y_lhp = x('l_hip_pitch');
    y_lkp = x('l_knee_pitch');
    y_rhp = x('r_hip_pitch');
    y_rkp = x('r_knee_pitch');  
    
    ya_2 = [y_lhp;
            y_lkp;
            y_rhp;
            y_rkp];
    
    % optional
    y2_label = {'LeftHipPitch',...
        'LeftKneePitch',...
        'RightHipPitch',...
        'RightKneePitch'};
    
    y2 = VirtualConstraint(domain,ya_2,'position','DesiredType','Bezier','PolyDegree',5,...
        'RelativeDegree',2,'OutputLabel',{y2_label},'PhaseType','TimeBased',...
        'PhaseVariable',tau,'PhaseParams',p,'Holonomic',true,'LoadPath',load_path);
    
    
    domain = addVirtualConstraint(domain,y2);
    
end
    