%% Start up robots
myDxl = slDxl('COM10', 1000000); % Replace with your COM port
findDxls(myDxl);
for idx = 1:5
    doEnableTorque(myDxl,idx);
end

%% Send commands
% Replace this part with inverse kinematics code
pos = zeros(1,5);
writePositions(myDxl,pos);
pause(3)
pos(5) = -pi/4;
writePositions(myDxl,pos);

%% Clean up
% NOTE: Disable torque only when it is safe!
for idx = 1:5
    doDisableTorque(myDxl,idx);
end
%release(myDxl);
%delete(myDxl);