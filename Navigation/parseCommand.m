function [goalPoint,objType] = parseCommand(speechStr)
% Simple parser that searches for object types and locations in a string

    % Find the object type
    if contains(speechStr,'blue')
        objType = 'blue';
    elseif contains(speechStr,'red')
        objType = 'red';
    else
        objType = '';
    end
    
    % Find the room
    if contains(speechStr,'kitchen')
        goalPoint = [1.5 -10];
    elseif contains(speechStr,'living')
        goalPoint = [-5 4];
    else
        goalPoint = [];
    end

end

