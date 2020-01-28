function p = get_mocap_position_from_message(msg)
    p = [msg.Pose.Position.X ; msg.Pose.Position.Y ; msg.Pose.Position.Z]./1000 ;
end
