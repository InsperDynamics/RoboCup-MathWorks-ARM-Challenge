function Gripper(coordinator, state)
       if strcmp(state,'on') == 1
           %% Ativar gripper
            pause(1);
            nextPart = coordinator.Parts{coordinator.NextPart};
            % Adicionar nova peça pegada na verificação de colisão
            partBody = getBody(coordinator.Robot,'pickedPart');
            if nextPart.type == 1
                addCollision(partBody,"mesh", 'exampleHelperBottlePoints.stl', trvec2tform([0 0 0.08])*axang2tform([1 0 0 pi]));
                addVisual(partBody,"mesh", 'exampleHelperBottlePoints.stl', trvec2tform([0 0 0.08])*axang2tform([1 0 0 pi]));
                % Atualizar visualização
                coordinator.WorldPatchHandles{nextPart.index}.Visible = 'off';
                show(coordinator.Robot, coordinator.CurrentRobotJConfig,'PreservePlot', false, 'Frames', 'off');
                drawnow;
                % Remover esta parte dos obstáculos
                coordinator.World(nextPart.index) = []; 
            else
                addCollision(partBody,"mesh", 'exampleHelperCanPoints.stl', trvec2tform([0 0 0.01])*axang2tform([1 0 0 pi]));
                addVisual(partBody,"mesh", 'exampleHelperCanPoints.stl', trvec2tform([0 0 0.01])*axang2tform([1 0 0 pi]));
                % Atualizar visualização
                coordinator.WorldPatchHandles{nextPart.index}.Visible = 'off';
                show(coordinator.Robot, coordinator.CurrentRobotJConfig,'PreservePlot', false, 'Frames', 'off');
                drawnow;
                % Remover esta parte dos obstáculos
                coordinator.World(nextPart.index) = []; 
            end            
            % Atualizar os índices das peças detectadas restantes
            for i = (coordinator.NextPart + 1):numel(coordinator.Parts)
                if coordinator.Parts{i}.index > nextPart.index 
                    coordinator.Parts{i}.index = coordinator.Parts{i}.index -1;
                end
            end            
            % Gerar comando de gripper
            [gripAct,gripGoal] = rosactionclient('/my_gen3/custom_gripper_controller/gripper_cmd');
            gripperCommand = rosmessage('control_msgs/GripperCommand');
            gripperCommand.Position = 0.04; % 0.04 fully closed, 0 fully open
            gripperCommand.MaxEffort = 500;
            gripGoal.Command = gripperCommand;            
            pause(1);            
            % Enviar comando de gripper
            sendGoal(gripAct,gripGoal); 
            disp('Gripper closed...');
       else
           %% Desativar gripper
            pause(1);
            % Remover a peça colocada da verificação de colisão
            partBody = getBody(coordinator.Robot,'pickedPart');
            clearCollision(partBody);
            clearVisual(partBody);
            % Atualizar visualização
            show(coordinator.Robot, coordinator.CurrentRobotJConfig,'PreservePlot', false, 'Frames', 'off');
            drawnow;
            % Gerar comando de gripper
            [gripAct,gripGoal] = rosactionclient('/my_gen3/custom_gripper_controller/gripper_cmd');
            gripperCommand = rosmessage('control_msgs/GripperCommand');
            gripperCommand.Position = 0.0; % 0.04 fully closed, 0 fully open
            gripperCommand.MaxEffort = 500;
            gripGoal.Command = gripperCommand;            
            pause(1);            
            % Enviar comando de gripper
            sendGoal(gripAct,gripGoal);
            disp('Gripper open...');
       end
       
       pause(2);
end