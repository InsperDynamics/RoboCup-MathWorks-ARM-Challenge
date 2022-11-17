rosIP = '192.168.241.129';
rosshutdown;
rosinit(rosIP,11311);

load('KINOVAGen3.mat'); 
rng(0)

initialRobotJConfig =  [3.5797   -0.6562   -1.2507   -0.7008    0.7303   -2.0500   -1.9053];
endEffectorFrame = "gripper";

coordinator = Coordinator(robot,initialRobotJConfig, endEffectorFrame);
coordinator.HomeRobotTaskConfig = getTransform(robot, initialRobotJConfig, endEffectorFrame);
coordinator.PlacingPose{1} = trvec2tform([0.2 0.55 0.26])*axang2tform([0 0 1 pi/2])*axang2tform([0 1 0 pi]);
coordinator.PlacingPose{2} = trvec2tform([0.2 -0.55 0.26])*axang2tform([0 0 1 pi/2])*axang2tform([0 1 0 pi]);

BuildWorld(coordinator);

MoveTo(coordinator,coordinator.HomeRobotTaskConfig);

DetectParts(coordinator);

remainingParts = PickingLogic(coordinator);

while remainingParts==true
    GraspPose(coordinator);

    MoveTo(coordinator, coordinator.GraspPose);
    
    Gripper(coordinator,'on');
    
    MoveTo(coordinator, ...
    coordinator.PlacingPose{coordinator.DetectedParts{coordinator.NextPart}.placingBelt});
    
    Gripper(coordinator,'off');
    
    remainingParts = PickingLogic(coordinator);    

    MoveTo(coordinator,coordinator.HomeRobotTaskConfig);
end