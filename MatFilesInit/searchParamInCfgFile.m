function value = searchParamInCfgFile(filename,paramname,paramType)

value = -1;

fid = fopen(filename);
if fid==-1
    return    
end
[C]=textscan(fid,'%s %s','CommentStyle','%');
fclose(fid);

params = C{1};
values = C{2};
for i=1:length(params)
    parameter = char(params(i));
    if parameter(1)=='[' && parameter(end)==']' && strcmpi(parameter(2:end-1),paramname)

        if strcmpi(paramType,'integer') || strcmpi(paramType,'double')
            value = str2double(values(i));
        elseif strcmpi(paramType,'string')
            value = values{i};
        elseif strcmpi(paramType,'bool')
            if strcmpi(values(i),'true')
                value = true;
            elseif strcmpi(values(i),'false')
                value = false;
            else
                error('Error: parameter %s must be a boolean.',params(i));
            end
        else
            error('Error in searchParamInCfgFile: paramType can be only integer, double, string, or bool.');
        end
               
        return
    end
end
