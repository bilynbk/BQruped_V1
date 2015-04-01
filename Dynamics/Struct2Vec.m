function [vec,names] = Struct2Vec(struct,varargin)
        
        if nargin > 1
            names = varargin{1};
            notFoundIndex = []; % keep track of the index-names that were not found in the struct
            vec = zeros(length(names),1);
            for i = 1:length(names)
                try % not sure if the value was set
                    vec(i) = eval(['struct.',names{i},';']);
                catch ME % if not, set the corresponding entry to 0;
                    if strcmp(ME.identifier,'MATLAB: nonExistentField');
                        notFoundIndex = [notFoundIndex, i];
                        vec(i) = 0;
                    end
                end
            end
            
            % issue a warning about the index-names that were not found in
            % the struct
            if ~isempty(notFoundIndex)
                warning('Struct2Vec:NameNotFound','No entries for the following names were found and set to 0: ');
                a = warning('query','Struct2Vec:NameNotFound');
                if ~strcmp(a.state,'off')
                    for i = length(notFoundIndex)
                        disp(['  ->''',char(names{notFoundIndex(i)}),'''']);
                    end
                end
            end
            
        else % no index-names provided, use all variables from the struct
            fNames = fieldnames(struct);
            c = 0;
            vec = [];
            % cycle through all names
            for i = 1:length(fNames)
                newEl = struct.(fNames{i});
                [m,n] = size(newEl);
                for j = 1:m
                    for k = 1:n
                        c = c+1;
                        vec(c) = newEl(j,k);
                        if m*n ==1 % scalar
                            names{c} = fNames{i};
                        else % Vector/Matrix:
                            names{c} = [fNames{i},'(',num2str(j),',',num2str(k),')'];
                        end
                    end
                end
            end
            vec = vec';
            
        end
        
        
end
