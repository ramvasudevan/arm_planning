function [R,T,J] = get_link_rotations_and_translations_from_arm_data(j_vals,j_axes,j_locs)
    % setup some info
    n = length(j_vals) ;
    d = 3 ;
    kinematic_chain = [0:(n-1) ; 1:n] ;

    % allocate cell arrays for the rotations and translations
    R = mat2cell(repmat(eye(d),1,n),d,d*ones(1,n)) ;
    T = mat2cell(repmat(zeros(d,1),1,n),d,ones(1,n)) ;

    % allocate array for the joint locations
    J = nan(d,n) ;

    % move through the kinematic chain and get the rotations and
    % translation of each link
    for idx = 1:n
        k_idx = kinematic_chain(:,idx) ;
        p_idx = k_idx(1) ;
        s_idx = k_idx(2) ;

        % get the rotation and translation of the predecessor and
        % successor links; we assume the baselink is always rotated
        % to an angle of 0 and with a translation of 0
        if p_idx == 0
            R_pred = eye(d) ;
            T_pred = zeros(d,1) ;
        else
            R_pred = R{p_idx} ;
            T_pred = T{p_idx} ;
        end

        % get the value and location of the current joint
        j_idx = j_vals(idx) ;
        j_loc = j_locs(:,idx) ;

        % compute link rotation matrix of current link
        axis_pred = R_pred*j_axes(:,idx) ;
        R_succ = axis_angle_to_rotation_matrix_3D([axis_pred', j_idx])*R_pred ;

        % create translation
        T_succ = T_pred + R_pred*j_loc(1:d) - R_succ*j_loc(d+1:end) ;

        % fill in rotation and translation cells
        R{s_idx} = R_succ ;
        T{s_idx} = T_succ ;

        % fill in the joint location
        j_loc_local = j_locs((d+1):end,idx) ;
        J(:,idx) = -R_succ*j_loc_local + T_succ ;
    end
end