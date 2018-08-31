function jntPos = JointMsgToStruct(robot, jntState)
    
jntPos = robot.homeConfiguration;

for i = 1:length(jntState.Name)
    idx = strcmp({jntPos.JointName}, jntState.Name{i});
    jntPos(idx).JointPosition = jntState.Position(i);
end

end