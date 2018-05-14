function varargout = interface(varargin)
% INTERFACE MATLAB code for interface.fig
%      INTERFACE, by itself, creates a new INTERFACE or raises the existing
%      singleton*.
%
%      H = INTERFACE returns the handle to a new INTERFACE or the handle to
%      the existing singleton*.
%
%      INTERFACE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in INTERFACE.M with the given input arguments.
%
%      INTERFACE('Property','Value',...) creates a new INTERFACE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before interface_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to interface_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help interface

% Last Modified by GUIDE v2.5 10-May-2018 20:11:25

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @interface_OpeningFcn, ...
                   'gui_OutputFcn',  @interface_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before interface is made visible.
function [] = interface_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to interface (see VARARGIN)

% Choose default command line output for interface
handles.output = hObject;

files = dir('../model/test_objects/*.mat');
set(handles.LB_files,'string',{files.name});

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes interface wait for user response (see UIRESUME)
% uiwait(handles.figure1);
addpath ../model
addpath ../model/test_objects
addpath ../model/results
addpath ../offline
addpath ../optimization
addpath ../optimization/autodiff_generated
addpath ../planning
addpath ../ploting
addpath ../snopt

clc

% --- Outputs from this function are returned to the command line.
function varargout = interface_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% ---------------------------------------------------------------------
%       User Defined Functionalities
% ---------------------------------------------------------------------




function [] = BTN_load_model_Callback(hObject, eventdata, handles)
global filename para fgraph pgraph mesh mesh_s grasps gripper q0 qf
clc;
load(filename);

% -----------------------------------------------
% 		Offline-computation
% -----------------------------------------------

% planning parameter
para.GRIPPER_TILT_LIMIT = 30*pi/180; % tilting angle tolerance

para.GRIPPER_Z_LIMIT    = 1; % 2 finger position limit
para.FINGER_OPEN_SPACE_0  = 18; % 15mm each side. used for checking collision with table
para.FINGER_OPEN_SPACE_f  = 16; % 15mm each side. used for checking collision with table
para.FINGER_RADIUS      = 9.5; % used for checking collision with table

para.MU                 = 1.1; % friction between object and the table
para.COM_ERR            = 3; % 2 uncertainties in COM measurement
para.GP_ERR             = 5; % 20 uncertainties in Grasp point measurement
									 
para.N_Grasps_Attempt  = 10; % number of grasps that we run planGripper on 
% optimization parameter
para.opt_obj_N               = 20;
para.opt_obj_con_delta_theta = 0.3;
para.opt_obj_cost_k_dq       = 2;
para.opt_obj_cost_k_tilt     = 1;

% ploting control
para.showCheckedGrasp     = false;
para.showCheckedGrasp_id  = 1;
para.showGraspChecking    = false;
para.showGraspChecking_id = [2 3];
para.printing 			  = true; % control any printing outside of 'interface.m'
para.scene_offset         = [72 312 250]'; % [0 0 0] in planning will be this point in the scene

% visualize all the checked grasps
if para.showCheckedGrasp
	disp('Visualizing all checked grasps:');
	plotObject(mesh, para.showCheckedGrasp_id); hold on;
	for i = 1:grasps.count
	    gp = reshape(grasps.points(:,i,:), [3,2]);
		plot3(gp(1,:), gp(2,:), gp(3,:), '. -','linewidth',2, 'markersize', 20);
		% drawnow;
	end
end


% Plot the object to GUI
q0 = [1 0 0 0]';
q0 = quatMTimes(aa2quat(pi/2, [-1; 0; 0]), q0);
qf = [1 0 0 0]';

plotObject(mesh, handles.AX_initial, q0);
plotObject(mesh, handles.AX_final, qf);

axis off
rotate3d on;


disp('[Load Model] Model is loaded.');

set(handles.BTN_plan, 'Enable', 'on');
set(handles.BTN_compare, 'Enable', 'on');
set(handles.BTN_compare2, 'Enable', 'on');
set(handles.BTN_ReadInitialPose, 'Enable', 'on');
set(handles.BTN_rand_initial, 'Enable', 'on');
set(handles.BTN_rand_final, 'Enable', 'on');



function [] = BTN_plan_Callback(hObject, eventdata, handles)
clc;
global para fgraph pgraph mesh gripper grasps q0 qf qg0 qgf% inputs
global path_found plan % outputs
global grasp_id_0 grasp_id_f

disp('[Planning] Planning begin.');

% get grasp points/grasp angles for initial and final pose
if get(handles.CB_auto_grasp0, 'Value')
	grasp_id_0 = [];
	qg0        = [];
end

if get(handles.CB_auto_graspf, 'Value')
	grasp_id_f = [];
	qgf        = [];
end

if get(handles.CB_pickplace, 'Value')
	method = 'pickplace';
else
	method = 'pivoting';
end


para.printing = true;

[path_found, plan] = solveAProblem(q0, qf, qg0, qgf, grasp_id_0, grasp_id_f, method);


if path_found
	set(handles.BTN_animate, 'Enable', 'on');
    set(handles.BTN_gentraj, 'Enable', 'on');
else
	set(handles.BTN_animate, 'Enable', 'off');
    set(handles.BTN_gentraj, 'Enable', 'off');
end


function [] = BTN_rand_initial_Callback(hObject, eventdata, handles)
global mesh q0 fgraph

% q0 = quatRand();

N = size(fgraph.quat,2);
q0 = fgraph.quat(:,randi(N));

plotObject(mesh, handles.AX_initial, q0);


function [] = BTN_rand_final_Callback(hObject, eventdata, handles)
global mesh qf q0 fgraph

% qf = quatRand();
% qf = quatMTimes(aa2quat(pi, [0 1 0]'), q0);
N = size(fgraph.quat,2);
qf = fgraph.quat(:,randi(N));

plotObject(mesh, handles.AX_final, qf);


function [] = BTN_animate_Callback(hObject, eventdata, handles)
global path_found plan

if path_found
	animatePlan(5, plan);
end


% --- Executes on selection change in LB_files.
function [] = LB_files_Callback(hObject, eventdata, handles)
global filename;
% hObject    handle to LB_files (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns LB_files contents as cell array
%        contents{get(hObject,'Value')} returns selected item from LB_files
contents = cellstr(get(hObject,'String'));
filename = contents{get(hObject,'Value')};


% --- Executes during object creation, after setting all properties.
function [] = LB_files_CreateFcn(hObject, eventdata, handles)
% hObject    handle to LB_files (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in BTN_rand_grasp0.
function [] = BTN_rand_grasp0_Callback(hObject, eventdata, handles)
global q0  % input
global qg0 grasp_id_0 % output
global grasps gripper mesh pgraph para
% hObject    handle to BTN_rand_grasp0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[grasp_id_0, qg0, gp10_w, gp20_w] = randPickGrasp(q0);
% load debug
% grasp_id_0 = find(grasp_id_0);
% gp1o_w = grasps.points(:, grasp_id_0, 1);
% gp2o_w = grasps.points(:, grasp_id_0, 2);
% gp10_w = quatOnVec(gp1o_w, q0);
% gp20_w = quatOnVec(gp2o_w, q0);

if isempty(grasp_id_0)
	return;
end
 
plotObject(mesh, handles.AX_initial, q0);
plotGripper(handles.AX_initial, gripper, q0, [gp10_w gp20_w], qg0);
rotate3d on;


% --- Executes on button press in BTN_rand_graspf.
function [] = BTN_rand_graspf_Callback(hObject, eventdata, handles)
global qf 
global qgf grasp_id_f
global grasps gripper mesh pgraph para
% hObject    handle to BTN_rand_grasp0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[grasp_id_f, qgf, gp1f_w, gp2f_w]      = randPickGrasp(qf);
% load debug
% grasp_id_f = find(grasp_id_f);
% gp1o_w = grasps.points(:, grasp_id_f, 1);
% gp2o_w = grasps.points(:, grasp_id_f, 2);
% gp1f_w = quatOnVec(gp1o_w, qf);
% gp2f_w = quatOnVec(gp2o_w, qf);

if isempty(grasp_id_f)
	return;
end
 
plotObject(mesh, handles.AX_final, qf);
plotGripper(handles.AX_final, gripper, qf, [gp1f_w gp2f_w], qgf);
rotate3d on;

% --- Executes on button press in CB_auto_grasp0.
function [] = CB_auto_grasp0_Callback(hObject, eventdata, handles)
% hObject    handle to CB_auto_grasp0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global q0 qg0
global mesh grasps pgraph gripper grasp_id_0 para
% Hint: get(hObject,'Value') returns toggle state of CB_auto_grasp0
if get(handles.CB_auto_grasp0, 'Value')
	set(handles.BTN_rand_grasp0, 'Enable', 'off');
	set(handles.Slider_grasp0, 'Enable', 'off');
	% re-plot just the object 
	plotObject(mesh, handles.AX_initial);
	rotate3d on;
else
	set(handles.BTN_rand_grasp0, 'Enable', 'on');
	set(handles.Slider_grasp0, 'Enable', 'on');
	% re-plot the object and gripper 
    % pick a grasp
    all_grasps          = checkGraspPoints(grasps, mesh, pgraph, q0, para.showGraspChecking_id(2), para);
    ids                 = find(all_grasps);
    id                  = randi(length(ids));
    grasp_id_0          = zeros(size(all_grasps));
    grasp_id_0(ids(id)) = 1;
	assert(sum(grasp_id_0) == 1);
	grasp_id           = find(grasp_id_0);
	gp1o_w             = grasps.points(:, grasp_id, 1);
	gp2o_w             = grasps.points(:, grasp_id, 2);
	gp10_w             = quatOnVec(gp1o_w, q0);
	gp20_w             = quatOnVec(gp2o_w, q0);
	gripper_cone_width = getTiltedGripperCone(gp10_w, gp20_w, para.GRIPPER_TILT_LIMIT);
	qg0                = getProperGrasp(grasp_id, q0, gripper_cone_width, false);
	
	plotObject(mesh, handles.AX_initial, q0);
	plotGripper(handles.AX_initial, gripper, q0, [gp10_w gp20_w], qg0);
	rotate3d on;
end

% --- Executes on button press in CB_auto_graspf.
function [] = CB_auto_graspf_Callback(hObject, eventdata, handles)
% hObject    handle to CB_auto_graspf (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global qf qgf
global mesh grasps gripper pgraph grasp_id_f para
% Hint: get(hObject,'Value') returns toggle state of CB_auto_grasp0
if get(handles.CB_auto_graspf, 'Value')
	set(handles.BTN_rand_graspf, 'Enable', 'off');
	set(handles.Slider_graspf, 'Enable', 'off');
	% re-plot just the object 
	plotObject(mesh, handles.AX_final);
	rotate3d on;
else
	set(handles.BTN_rand_graspf, 'Enable', 'on');
	set(handles.Slider_graspf, 'Enable', 'on');
	% re-plot the object and gripper 
	
    % pick a grasp
    all_grasps          = checkGraspPoints(grasps, mesh, pgraph, qf, para.showGraspChecking_id(2), para);
    ids                 = find(all_grasps);
    id                  = randi(length(ids));
    grasp_id_f          = zeros(size(all_grasps));
    grasp_id_f(ids(id)) = 1;
    
	assert(sum(grasp_id_f) == 1);
	grasp_id           = find(grasp_id_f);
	gp1o_w             = grasps.points(:, grasp_id, 1);
	gp2o_w             = grasps.points(:, grasp_id, 2);
	gp1f_w             = quatOnVec(gp1o_w, qf);
	gp2f_w             = quatOnVec(gp2o_w, qf);
	gripper_cone_width = getTiltedGripperCone(gp1f_w, gp2f_w, para.GRIPPER_TILT_LIMIT);
	qgf                = getProperGrasp(grasp_id, qf, gripper_cone_width, false);

	plotObject(mesh, handles.AX_final, qf);
	plotGripper(handles.AX_final, gripper, qf, [gp1f_w gp2f_w], qgf);
	rotate3d on;
end

% --- Executes on slider movement.
function [] = Slider_grasp0_Callback(hObject, eventdata, handles)
% hObject    handle to Slider_grasp0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function [] = Slider_grasp0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Slider_grasp0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function [] = Slider_graspf_Callback(hObject, eventdata, handles)
% hObject    handle to Slider_graspf (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function [] = Slider_graspf_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Slider_graspf (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% 	Re-orienting test
function [] = BTN_compare_Callback(hObject, eventdata, handles)
clc;
global para fgraph pgraph mesh gripper grasps % inputs

para.printing = false;
finger_open_range = [5 8 10 11 12 13 14 17];
Nexp = length(finger_open_range);

% ------------------------------------------
% 	get list of objects
% ------------------------------------------
files = dir('../model/test_objects/*.mat');
Nobj  = length(files);


% pivoting.plans       = cell(1, Nobj);
% pivoting.scores      = cell(1, Nobj);
% pivoting.path_found  = [];
% pickplace.plans      = cell(1, Nobj);
% pickplace.scores     = cell(1, Nobj);
% pickplace.path_found = [];
pivoting.path_found  = cell(1, Nexp);
pickplace.path_found = cell(1, Nexp);

for ex = 1:Nexp
	para.FINGER_OPEN_SPACE = finger_open_range(ex);
	pivoting.path_found{ex} = [];
	pickplace.path_found{ex} = [];

	disp(['[Finger Open Space: ' num2str(para.FINGER_OPEN_SPACE) ']' ]);

	for i_obj = 1:Nobj
		load(files(i_obj).name);
		disp(['[Object #' num2str(i_obj) '] ---- ' files(i_obj).name ' ----']);

		% list of problems
		C    = nchoosek(1:fgraph.NM, 2);
		NP_i = size(C, 1); % number of problems for this object
		if NP_i > 30
			NP_i = 30;
			C    = C(randi(NP_i, [1 30]), :);
		end
		disp(['[Stable Modes: ' num2str(fgraph.NM) '	Problems: ' num2str(NP_i)]);

		pivoting.plans{i_obj}       = cell(1, NP_i);      
		pivoting.scores{i_obj}      = cell(1, NP_i);     
		pickplace.plans{i_obj}      = cell(1, NP_i);     
		pickplace.scores{i_obj}     = cell(1, NP_i);    

		pivoting_path_found  = zeros(1, NP_i);
		pickplace_path_found = zeros(1, NP_i);

		for p = 1:NP_i
			mid0 = C(p, 1);
			midf = C(p, 2);

			q0 = fgraph.quat(:, mid0);
			qf = fgraph.quat(:, midf);

			[pivoting_path_found(p), ~]   = solveAProblem(q0, qf, [], [], [], [], 'pivoting');
			[pickplace_path_found(p), ~] = solveAProblem(q0, qf, [], [], [], [], 'pickplace');

			if pickplace_path_found(p) && ~pivoting_path_found(p)
				error('weird');
			end

			% pivoting.scores{i_obj}{p}  = evalPlan(pivoting.plans{i_obj}{p});
			% pickplace.scores{i_obj}{p} = evalPlan(pickplace.plans{i_obj}{p});

			disp(['		Problem #' num2str(p) ' of ' num2str(NP_i) ', Pivoting: ' num2str(pivoting_path_found(p)) ', P&P: ' num2str(pickplace_path_found(p))]);
		end

		pivoting.path_found{ex}  = [pivoting.path_found{ex} pivoting_path_found];
		pickplace.path_found{ex} = [pickplace.path_found{ex} pickplace_path_found];
	end
end

save '../results/comparison1.mat' pivoting pickplace finger_open_range;
set(handles.BTN_show_results, 'Enable', 'on');





% --- Executes on button press in BTN_show_results.
function [] = BTN_show_results_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_show_results (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
load '../results/comparison1.mat'
clc;

Nexp = length(finger_open_range);

N_pivoting_found  =  zeros(Nexp, 1);
N_pickplace_found =  zeros(Nexp, 1);
for i = 1:Nexp
	N_pivoting_found(i)  = sum(pivoting.path_found{i});
	N_pickplace_found(i) = sum(pickplace.path_found{i});
	disp(['Finger open dist: ' num2str(finger_open_range(i)) ', NFound: ' num2str(N_pivoting_found(i)) '	' num2str(N_pickplace_found(i))] );
end

figure(1);clf(1);hold on;
title('# of solutions');
plot(finger_open_range, N_pivoting_found,'*-b');
plot(finger_open_range, N_pickplace_found,'*-r');
xlabel('finger width (mm)');

% pivoting_ex_time      = zeros(1, N);
% pivoting_rotation     = zeros(1, N);
% pivoting_translation  = zeros(1, N);
% pivoting_NRegrasp     = zeros(1, N);
% pickplace_ex_time     = zeros(1, N);
% pickplace_rotation    = zeros(1, N);
% pickplace_translation = zeros(1, N);
% pickplace_NRegrasp    = zeros(1, N);

% pivoting_ex_time_total      = 0;
% pivoting_rotation_total     = 0;
% pivoting_translation_total  = 0;
% pivoting_NRegrasp_total     = 0;
% pickplace_ex_time_total     = 0;
% pickplace_rotation_total    = 0;
% pickplace_translation_total = 0;
% pickplace_NRegrasp_total    = 0;

% Nobj = length(pivoting.plans);
% count = 0;
% for i_obj = 1:Nobj
% 	for i = 1:length(pivoting.plans{i_obj})
% 		count = count + 1;
% 		if pivoting.path_found(count)
% 			pivoting_ex_time(count)      = pivoting.scores{i_obj}{i}.execution_time;
% 			pivoting_rotation(count)     = pivoting.scores{i_obj}{i}.gripper_rotation;        
% 			pivoting_translation(count)  = pivoting.scores{i_obj}{i}.gripper_translation;  
% 			pivoting_NRegrasp(count)     = pivoting.scores{i_obj}{i}.N_regrasp;
% 		end
% 		if pickplace.path_found(count)
% 			pickplace_ex_time(count)     = pickplace.scores{i_obj}{i}.execution_time;
% 			pickplace_rotation(count)    = pickplace.scores{i_obj}{i}.gripper_rotation;
% 			pickplace_translation(count) = pickplace.scores{i_obj}{i}.gripper_translation;  
% 			pickplace_NRegrasp(count)    = pickplace.scores{i_obj}{i}.N_regrasp;

% 			pivoting_ex_time_total      = pivoting_ex_time_total      + pivoting_ex_time(count);
% 			pivoting_rotation_total     = pivoting_rotation_total     + pivoting_rotation(count);
% 			pivoting_translation_total  = pivoting_translation_total  + pivoting_translation(count);
% 			pivoting_NRegrasp_total     = pivoting_NRegrasp_total     + pivoting_NRegrasp(count);
% 			pickplace_ex_time_total     = pickplace_ex_time_total     + pickplace_ex_time(count);
% 			pickplace_rotation_total    = pickplace_rotation_total    + pickplace_rotation(count);
% 			pickplace_translation_total = pickplace_translation_total + pickplace_translation(count);
% 			pickplace_NRegrasp_total    = pickplace_NRegrasp_total    + pickplace_NRegrasp(count);
% 		end
% 	end
% end

% pivoting_ex_time_aver      = pivoting_ex_time_total/N_pickplace_found;
% pivoting_rotation_aver     = pivoting_rotation_total/N_pickplace_found;
% pivoting_translation_aver  = pivoting_translation_total/N_pickplace_found;
% pivoting_NRegrasp_aver     = pivoting_NRegrasp_total/N_pickplace_found;
% pickplace_ex_time_aver     = pickplace_ex_time_total/N_pickplace_found;
% pickplace_rotation_aver    = pickplace_rotation_total/N_pickplace_found;
% pickplace_translation_aver = pickplace_translation_total/N_pickplace_found;
% pickplace_NRegrasp_aver    = pickplace_NRegrasp_total/N_pickplace_found;

% disp(['Total rotation: ' num2str(sum(pivoting_rotation_aver)) '		' num2str(sum(pickplace_rotation_aver))] );
% disp(['Total translation: ' num2str(sum(pivoting_translation_aver)) '		' num2str(sum(pickplace_translation_aver))] );
% disp(['# of regrasp: ' num2str(sum(pivoting_NRegrasp_aver)) '		' num2str(sum(pickplace_NRegrasp_aver))] );




% --- Executes on button press in BTN_compare2.
function [] = BTN_compare2_Callback(hObject, eventdata, handles)
clc;
global para fgraph pgraph mesh gripper grasps % inputs

para.printing = false;
N_sample_per_object = 100;

% ------------------------------------------
% 	get list of objects
% ------------------------------------------
files = dir('../model/data/*.mat');
Nobj  = length(files);

tilt_limit_array = [1:8]*10*pi/180;

for experiment = 1:length(tilt_limit_array)
    tic;
	para.GRIPPER_TILT_LIMIT          = tilt_limit_array(experiment); % tilting angle tolerance
	pivoting.plans       = cell(1, Nobj);
	pivoting.scores      = cell(1, Nobj);
	pivoting.path_found  = [];
	pickplace.plans      = cell(1, Nobj);
	pickplace.scores     = cell(1, Nobj);
	pickplace.path_found = [];

	for i_obj = 2:Nobj
		load(files(i_obj).name);
		disp(['[Object #' num2str(i_obj) '] ---- ' files(i_obj).name ' ----']);

		pivoting.plans{i_obj}       = cell(1, N_sample_per_object);      
		pivoting.scores{i_obj}      = cell(1, N_sample_per_object);     
		pickplace.plans{i_obj}      = cell(1, N_sample_per_object);     
		pickplace.scores{i_obj}     = cell(1, N_sample_per_object);    

		pivoting_path_found  = zeros(1, N_sample_per_object);
		pickplace_path_found = zeros(1, N_sample_per_object);

		for p = 1:N_sample_per_object
			while true
				q0                = quatRand();
				qf                = quatRand();
				[grasp_id_0, qg0] = randPickGrasp(q0);
				[grasp_id_f, qgf] = randPickGrasp(qf);
				if (~isempty(grasp_id_0))&&(~isempty(grasp_id_f))
					break;
				end
			end

			[pivoting_path_found(p), pivoting.plans{i_obj}{p}]   = solveAProblem(q0, qf, qg0, qgf, grasp_id_0, grasp_id_f, 'pivoting');
			[pickplace_path_found(p), pickplace.plans{i_obj}{p}] = solveAProblem(q0, qf, qg0, qgf, grasp_id_0, grasp_id_f, 'pickplace');

% 			if pickplace_path_found(p) && ~pivoting_path_found(p)
% 					warning('weird');
% 			end

			pivoting.scores{i_obj}{p}  = evalPlan(pivoting.plans{i_obj}{p});
			pickplace.scores{i_obj}{p} = evalPlan(pickplace.plans{i_obj}{p});

			disp(['		Problem #' num2str(p) ' of ' num2str(N_sample_per_object) ', Pivoting: ' num2str(pivoting_path_found(p)) ', P&P: ' num2str(pickplace_path_found(p))]);
		end

		pivoting.path_found  = [pivoting.path_found pivoting_path_found];
		pickplace.path_found = [pickplace.path_found pickplace_path_found];
    end
    toc; return;

    full_path = ['../model/results/comparison' num2str(experiment) '.mat'];
	save(full_path, 'pivoting', 'pickplace');
	set(handles.BTN_show_results, 'Enable', 'on');

end % end all experiments

% --- Executes on button press in BTN_show_results2.
function [] = BTN_show_results2_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_show_results2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in BTN_gentraj.
function [] = BTN_gentraj_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_gentraj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global path_found plan

if path_found
	generateRobotTraj(plan);
end


% --- Executes on button press in CB_pickplace.
function CB_pickplace_Callback(hObject, eventdata, handles)
% hObject    handle to CB_pickplace (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of CB_pickplace


% --- Executes on button press in BTN_ReadInitialPose.
function BTN_ReadInitialPose_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_ReadInitialPose (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global mesh q0

pose = dlmread('initial_pose.txt');
q0 = pose(4:7)';
plotObject(mesh, handles.AX_initial, q0);


% --- Executes on button press in BTN_EXP_Reset.
function BTN_EXP_Reset_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_Reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global reset_client
disp('Calling Reset_service:');
call(reset_client);
disp('Reset is done.');

% --- Executes on button press in BTN_EXP_read_obj_pose.
function BTN_EXP_read_obj_pose_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_read_obj_pose (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global read_obj_pose_client para
global mesh q0 qf

disp('Calling read_obj_pose service:');
call(read_obj_pose_client);

pose = dlmread('initial_pose.txt');
q0   = reshape(pose(4:7), [4,1]);
pt0  = quat2m(q0)*(mesh.vertices');
z0   = min(pt0(3,:));
para.scene_offset = 1000*pose(1:3)';
para.scene_offset(3) = para.scene_offset(3) + z0;


plotObject(mesh, handles.AX_initial, q0);
plotTable(handles.AX_initial);

% qf = quatMTimes(aa2quat(pi, [1 1 0]'), q0);
% qf = [0.5 0.5 0.5 0.5]';
% plotObject(mesh, handles.AX_final, qf);
% axis off
% rotate3d on;



set(handles.BTN_EXP_Planning, 'Enable', 'on');
set(handles.BTN_EXP_Pre_Grasp, 'Enable', 'off');
set(handles.BTN_EXP_Run, 'Enable', 'off');
set(handles.BTN_EXP_Release_Reset, 'Enable', 'off');

disp('Read obj pose is done.');

% --- Executes on button press in BTN_EXP_Planning.
function BTN_EXP_Planning_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_Planning (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clc;
global para fgraph pgraph mesh gripper grasps q0 qf qg0 qgf% inputs
global path_found plan % outputs
global grasp_id_0 grasp_id_f
global nseq
% -------------------------------------------------------
% 		Planning
% -------------------------------------------------------

disp('[Planning] Planning begin.');
% get grasp points/grasp angles for initial and final pose
if get(handles.CB_auto_grasp0, 'Value')
	grasp_id_0 = [];
	qg0        = [];
end

if get(handles.CB_auto_graspf, 'Value')
	grasp_id_f = [];
	qgf        = [];
end

if get(handles.CB_pickplace, 'Value')
	method = 'pickplace';
else
	method = 'pivoting';
end


para.printing = true;

[path_found, plan] = solveAProblem(q0, qf, qg0, qgf, grasp_id_0, grasp_id_f, method);


if path_found
	set(handles.BTN_animate, 'Enable', 'on');
    set(handles.BTN_EXP_Pre_Grasp, 'Enable', 'on');
    nseq = 1;
else
	set(handles.BTN_animate, 'Enable', 'off');
    set(handles.BTN_EXP_Pre_Grasp, 'Enable', 'off');
    return;
end

% -------------------------------------------------------
% 		Generate files
% -------------------------------------------------------
disp('Generating files:');
generateRobotTraj(plan);

disp('Planning is done.')

% --- Executes on button press in BTN_EXP_Pre_Grasp.
function BTN_EXP_Pre_Grasp_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_Pre_Grasp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global pre_grasp_client
disp('Calling Pre_grasp_service:');
call(pre_grasp_client);

set(handles.BTN_EXP_Init_ROS, 'Enable', 'off');
set(handles.BTN_EXP_Reset, 'Enable', 'off');
set(handles.BTN_EXP_read_obj_pose, 'Enable', 'off');
set(handles.BTN_EXP_Planning, 'Enable', 'off');
set(handles.BTN_NextTraj, 'Enable', 'off');
set(handles.BTN_EXP_Pre_Grasp, 'Enable', 'off');
set(handles.BTN_EXP_Run, 'Enable', 'on');
set(handles.BTN_EXP_Release_Reset, 'Enable', 'on');

disp('Pre Grasp is done.');

% --- Executes on button press in BTN_EXP_Run.
function BTN_EXP_Run_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_Run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global run_client nseq
disp('Calling Run_service:');
call(run_client);

nseq = nseq + 1;

set(handles.BTN_EXP_Init_ROS, 'Enable', 'off');
set(handles.BTN_EXP_Reset, 'Enable', 'off');
set(handles.BTN_EXP_read_obj_pose, 'Enable', 'off');
set(handles.BTN_EXP_Planning, 'Enable', 'off');
set(handles.BTN_EXP_Pre_Grasp, 'Enable', 'off');
set(handles.BTN_EXP_Run, 'Enable', 'off');
set(handles.BTN_EXP_Release_Reset, 'Enable', 'on');

disp('Run is done.');

% --- Executes on button press in BTN_EXP_Release_Reset.
function BTN_EXP_Release_Reset_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_Release_Reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global release_reset_client plan
call(release_reset_client);

set(handles.BTN_EXP_Init_ROS, 'Enable', 'off');
set(handles.BTN_EXP_Reset, 'Enable', 'on');
set(handles.BTN_EXP_read_obj_pose, 'Enable', 'on');
set(handles.BTN_EXP_Planning, 'Enable', 'off');
set(handles.BTN_EXP_Pre_Grasp, 'Enable', 'off');
set(handles.BTN_EXP_Run, 'Enable', 'off');
set(handles.BTN_EXP_Release_Reset, 'Enable', 'off');

if length(plan) > 1
    set(handles.BTN_NextTraj, 'Enable', 'on');
end
    
disp('Release_reset is done.');

% --- Executes on button press in BTN_EXP_Init_ROS.
function BTN_EXP_Init_ROS_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_Init_ROS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global reset_client read_obj_pose_client pre_grasp_client run_client release_reset_client
% rosinit;

reset_client         = rossvcclient('/regrasping/reset');
read_obj_pose_client = rossvcclient('/regrasping/read_obj_pose');
pre_grasp_client     = rossvcclient('/regrasping/pre_grasp');
run_client           = rossvcclient('/regrasping/run');
release_reset_client = rossvcclient('/regrasping/release_reset');

set(handles.BTN_EXP_Init_ROS, 'Enable', 'off');
set(handles.BTN_EXP_Reset, 'Enable', 'on');
set(handles.BTN_EXP_read_obj_pose, 'Enable', 'on');
set(handles.BTN_EXP_Planning, 'Enable', 'off');
set(handles.BTN_EXP_Pre_Grasp, 'Enable', 'off');
set(handles.BTN_EXP_Run, 'Enable', 'off');
set(handles.BTN_EXP_Release_Reset, 'Enable', 'off');

disp('Initialization is done. Service clients are ready to use.');


% --- Executes on button press in BTN_NextTraj.
function BTN_NextTraj_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_NextTraj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global plan nseq q0 mesh para grasps

assert(nseq > 1);
% Now the previous part of traj is finished.
% Find the current object pose
q0 = plan{nseq-1}.qobj(:, end); % this is the current obj q
gp1o_w = grasps.points(:, plan{nseq-1}.grasp_id, 1);
gp2o_w = grasps.points(:, plan{nseq-1}.grasp_id, 2);
gpo_w = (gp1o_w + gp2o_w)/2;
gp0_w = quatOnVec(gpo_w, q0); % this is the grp pos when p_obj = [0 0 0]

p_grp = dlmread('current_gripper_pose.txt');
p_grp = p_grp(1:3)'; % this is the current grp pos
p_obj = p_grp - gp0_w; % this is the current obj pos

pt0  = quat2m(q0)*(mesh.vertices');
z0   = min(pt0(3,:));
para.scene_offset = p_obj;
para.scene_offset(3) = para.scene_offset(3) + z0;
plotObject(mesh, handles.AX_initial, q0); hold on;
plotTable(handles.AX_initial);

disp(['Generating files for trajectory ' num2str(nseq) ':']);
generateRobotTraj({plan{nseq}});

disp('Done.')

set(handles.BTN_animate, 'Enable', 'on');
set(handles.BTN_EXP_Pre_Grasp, 'Enable', 'on');


