[~,python_locations]=system('where python');
python_paths=splitlines(python_locations);

flag=false;
for ip=1:length(python_paths)
    if contains(python_paths{ip},'Microsoft')
        warning(python_paths{ip}+" is not supported")
        continue
    elseif isempty(python_paths{ip})
        continue
    end
    pyenv('Version', python_paths{ip});
    flag=true;
    break
end
if not(flag)
    error('unable to find a valid version of python')
end
insert(py.sys.path, int32(0), './labauto')