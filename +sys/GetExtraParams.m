function [params] = GetExtraParams()
    % get parameters of Art3, jun 20180507
    
    % from knee to foot_ankle (aka point_foot)
    params.ka = 0.474;
    
    % friction about
    params.mu = 0.6;        % friction coefficient
    params.gamma = 100;     % rotational friction coefficient
    
    % flat foot (sole) about, maybe used in the future ~
    params.lt = 0.1728;
    params.lh = 0.082;
    params.wf = 0.1524;
    params.hf = -0.07645;
end
