function robot_disp = LoadRobotDisplay(robot, varargin)
    
    % jun, 20180508  ----> add Art3-2D's 4-Bars 
    
    
    ip = inputParser;   % Input parser for functions
    ip.addParameter('UseExported',true,@(x) isequal(x,true) || isequal(x,false));
    ip.addParameter('ExportPath','',@(x)ischar(x));
    ip.addParameter('SkipExporting',false,@(x) isequal(x,true) || isequal(x,false));
    
    ip.parse(varargin{:});
    
    opts = ip.Results;
    if isempty(opts.ExportPath)
        root_path = utils.get_root_path();
        export_path = fullfile(root_path,'gen','animator');
        opts.ExportPath = export_path;
    else
        export_path = opts.ExportPath;
    end
    if ~exist(export_path,'dir')
        mkdir(export_path);
    end
    addpath(export_path);
    
    f = figure(1000);clf;
    robot_disp = frost.Animator.Display(f, robot, opts);
    
    % add torso
    frame  = robot.Joints(3);   % 'BaseRotY'
    pelvis = robot.Links(getLinkIndices(robot,'pelvis'));    
    offset = pelvis.Offset;    
    torso_bar = frost.Animator.Cylinder(robot_disp.axs, robot, frame, offset, 'TorsoBar', opts);
    robot_disp.addItem(torso_bar);
    
    torso = frost.Animator.LinkSphere(robot_disp.axs, robot, pelvis, 'TorsoCoM', opts);
    torso.radius = 0.1;
    robot_disp.addItem(torso);
    
    % add l_FB_rod
    frame  = robot.Joints(getJointIndices(robot,'l_FB_2')); 
    offset = [0,0,-0.46];    
    l_FB_rod_bar = frost.Animator.Cylinder(robot_disp.axs, robot, frame, offset, 'l_FB_rod_bar', opts);
    robot_disp.addItem(l_FB_rod_bar);
    
    % add r_FB_rod
    frame  = robot.Joints(getJointIndices(robot,'r_FB_2'));
    offset = [0,0,-0.46];      
    r_FB_rod_bar = frost.Animator.Cylinder(robot_disp.axs, robot, frame, offset, 'r_FB_rod_bar', opts);
    robot_disp.addItem(r_FB_rod_bar);
    
    % add l_calf
    frame  = robot.Joints(getJointIndices(robot,'l_FB_1'));  
    offset = [0,0,-0.474];    
    l_calf_bar = frost.Animator.Cylinder(robot_disp.axs, robot, frame, offset, 'l_calf_bar', opts);
    robot_disp.addItem(l_calf_bar);
    
    % add r_calf
    frame  = robot.Joints(getJointIndices(robot,'r_FB_1'));
    offset = [0,0,-0.474];     
    r_calf_bar = frost.Animator.Cylinder(robot_disp.axs, robot, frame, offset, 'r_calf_bar', opts);
    robot_disp.addItem(r_calf_bar);
    
    % add feet
    left_foot = sys.frames.LeftPoint(robot);
    name = 'LeftFoot';
    left_foot = frost.Animator.LinkSphere(robot_disp.axs, robot, left_foot, name, opts);
    robot_disp.addItem(left_foot); 
    
    right_foot = sys.frames.RightPoint(robot);
    name = 'RightFoot';
    right_foot = frost.Animator.LinkSphere(robot_disp.axs, robot, right_foot, name, opts);
    robot_disp.addItem(right_foot);
    
    
end