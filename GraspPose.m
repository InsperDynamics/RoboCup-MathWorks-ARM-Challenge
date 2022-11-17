function GraspPose(coordinator) 
        nextPart = coordinator.Parts{coordinator.NextPart};
        if nextPart.type==2
            coordinator.GraspPose = trvec2tform(nextPart.centerPoint + [0 0 0.03])*axang2tform([0 1 0 pi])*axang2tform([0,0,1,-pi/2]);
        else
            coordinator.GraspPose = trvec2tform(nextPart.centerPoint + [0 0 0.06])*axang2tform([0 1 0 pi])*axang2tform([0,0,1, -pi/2]);
        end
end
