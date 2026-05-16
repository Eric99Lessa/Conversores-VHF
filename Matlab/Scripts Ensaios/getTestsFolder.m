function [testsFolder] = getTestsFolder(currentFolder)
    testsFolder = "";
    while testsFolder == "" && currentFolder ~= "/"
        [parentFolder, ~, ~] = fileparts(currentFolder);
        file_list = dir(parentFolder);
    
        % Filter for directories only
        is_dir_flag = [file_list.isdir]; % A logical vector
        folders = file_list(is_dir_flag);
    
        folder_names = {folders(3:end).name}; % Start at 3 to skip . and ..
        if any(folder_names == "Ensaios")
            testsFolder = parentFolder + "/Ensaios/";
        else
            currentFolder = parentFolder;
        end
    end
end