Flag_vector=[];
numFiles = 10;
%% Find infesible cases
for k = 1 : numFiles
    
    state_file=strcat('StatesOfvehicles_',num2str(k),'.mat');
    state_struct=load(state_file);
    state_cell=struct2cell(state_struct);
    state_cell_cell=state_cell{1};
    
    state_vehicle1=state_cell_cell{1}
    state_vehicle2=state_cell_cell{2}
end