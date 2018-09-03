function callback(~,msg)
    global pose
    
    pose.X = msg.X;
    pose.Y = msg.Y;
    pose.Theta = msg.Theta;
    
end