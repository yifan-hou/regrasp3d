function showResults()
% hObject    handle to BTN_show_results2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% load 'comparison8.mat'
clc;

for experiment = 1:8
    filename = ['comparison' num2str(experiment) '.mat'];
    load(filename);
    N                 = length(pivoting.path_found);
    N_pivoting_found  = sum(pivoting.path_found);
    N_pickplace_found = sum(pickplace.path_found);
    N
    disp(['NFound: ' num2str(N_pivoting_found) '	' num2str(N_pickplace_found)] );

    pivoting_ex_time      = zeros(1, N);
    pivoting_rotation     = zeros(1, N);
    pivoting_translation  = zeros(1, N);
    pivoting_NRegrasp     = zeros(1, N);
    pickplace_ex_time     = zeros(1, N);
    pickplace_rotation    = zeros(1, N);
    pickplace_translation = zeros(1, N);
    pickplace_NRegrasp    = zeros(1, N);

    pivoting_ex_time_total      = 0;
    pivoting_rotation_total     = 0;
    pivoting_translation_total  = 0;
    pivoting_NRegrasp_total     = 0;
    pickplace_ex_time_total     = 0;
    pickplace_rotation_total    = 0;
    pickplace_translation_total = 0;
    pickplace_NRegrasp_total    = 0;

    Nobj = length(pivoting.plans);
    count = 0;
    for i_obj = 1:Nobj
        for i = 1:length(pivoting.plans{i_obj})
            count = count + 1;
            if pivoting.path_found(count)
                pivoting_ex_time(count)     = pivoting.scores{i_obj}{i}.execution_time;
                pivoting_rotation(count)    = pivoting.scores{i_obj}{i}.gripper_rotation;        
                pivoting_translation(count) = pivoting.scores{i_obj}{i}.gripper_translation;  
                pivoting_NRegrasp(count)    = pivoting.scores{i_obj}{i}.N_regrasp;
                pivoting_ex_time_total      = pivoting_ex_time_total      + pivoting_ex_time(count);
                pivoting_rotation_total     = pivoting_rotation_total     + pivoting_rotation(count);
                pivoting_translation_total  = pivoting_translation_total  + pivoting_translation(count);
                pivoting_NRegrasp_total     = pivoting_NRegrasp_total     + pivoting_NRegrasp(count);
            end
            if pickplace.path_found(count)
                pickplace_ex_time(count)     = pickplace.scores{i_obj}{i}.execution_time;
                pickplace_rotation(count)    = pickplace.scores{i_obj}{i}.gripper_rotation;
                pickplace_translation(count) = pickplace.scores{i_obj}{i}.gripper_translation;  
                pickplace_NRegrasp(count)    = pickplace.scores{i_obj}{i}.N_regrasp;
                pickplace_ex_time_total      = pickplace_ex_time_total     + pickplace_ex_time(count);
                pickplace_rotation_total     = pickplace_rotation_total    + pickplace_rotation(count);
                pickplace_translation_total  = pickplace_translation_total + pickplace_translation(count);
                pickplace_NRegrasp_total     = pickplace_NRegrasp_total    + pickplace_NRegrasp(count);
            end
        end
    end

    pivoting_ex_time_aver      = pivoting_ex_time_total/N_pivoting_found;
    pivoting_rotation_aver     = pivoting_rotation_total/N_pivoting_found;
    pivoting_translation_aver  = pivoting_translation_total/N_pivoting_found;
    pivoting_NRegrasp_aver     = pivoting_NRegrasp_total/N_pivoting_found;
    pickplace_ex_time_aver     = pickplace_ex_time_total/N_pickplace_found;
    pickplace_rotation_aver    = pickplace_rotation_total/N_pickplace_found;
    pickplace_translation_aver = pickplace_translation_total/N_pickplace_found;
    pickplace_NRegrasp_aver    = pickplace_NRegrasp_total/N_pickplace_found;

    disp(['EX time: ' num2str(sum(pivoting_ex_time_aver)) '		' num2str(sum(pickplace_ex_time_aver))] );
    disp(['Total rotation: ' num2str(sum(pivoting_rotation_aver)) '		' num2str(sum(pickplace_rotation_aver))] );
    disp(['Total translation: ' num2str(sum(pivoting_translation_aver)) '		' num2str(sum(pickplace_translation_aver))] );
    disp(['# of regrasp: ' num2str(sum(pivoting_NRegrasp_aver)) '		' num2str(sum(pickplace_NRegrasp_aver))] );
end
