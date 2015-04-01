function struct = Vec2Struct(vec,names)
%Check the input validation
    if ((length(vec)) ~= length(names))
        error('Vec2Struct: Length miss match. Please make sure vector and name array have the same length');
    end
    %start with an empty struct
    struct = [];
    
    % creat struct one by one
    for i = 1:length(vec) 
        eval(['struct.',names{i},' = vec(i);']) % use eval command to create the struct
    end
end

