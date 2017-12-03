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

% Last Modified by GUIDE v2.5 14-Sep-2017 00:20:06

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
function interface_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to interface (see VARARGIN)

% Choose default command line output for interface
handles.output = hObject;

files = dir('../model/data/*.mat');
set(handles.LB_files,'string',{files.name});

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes interface wait for user response (see UIRESUME)
% uiwait(handles.figure1);
addpath ../planning
addpath ../model
addpath ../model/data
addpath ../model/results
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




function BTN_load_model_Callback(hObject, eventdata, handles)
global filename para fgraph pgraph mesh mesh_s grasps gripper q0 qf
clc;
load(filename);

% -----------------------------------------------
% 		Offline-computation
% -----------------------------------------------
% friction between object and  ground
para.MU = 0.5;

% planning parameter
para.GRIPPER_TILT_LIMIT          = 40*pi/180; % tilting angle tolerance
para.GRIPPER_Z_LIMIT             = 0.2; % finger position limit
para.PIVOTABLE_CHECK_GRANULARITY = 1*pi/180; % 1 sample every 1 degree
para.COLLISION_FREE_ANGLE_MARGIN = 5; % stay away from collsion for at least 5 degrees
									  % has to be an positive integer
% optimization parameter
para.opt_obj_N               = 20;
para.opt_obj_con_delta_theta = 0.3;
para.opt_obj_cost_k_dq       = 2;
para.opt_obj_cost_k_tilt     = 1;

% ploting control
para.showCheckedGrasp     = true;
para.showCheckedGrasp_id  = 1;
para.showGraspChecking    = false;
para.showGraspChecking_id = [2 3];
para.showProblem          = false;
para.showProblem_id       = [1 2 3];
para.show2Dproblem        = false;
para.show2Dproblem_id     = 4;
para.printing 			  = true; % control any printing outside of 'interface.m'


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
plotObject(mesh, handles.AX_initial);
plotObject(mesh, handles.AX_final);
plotObject(mesh, 2); % temporary 
axis off
rotate3d on;

q0 = [1 0 0 0]';
qf = [1 0 0 0]';

disp('[Load Model] Model is loaded.');

set(handles.BTN_plan, 'Enable', 'on');
set(handles.BTN_compare, 'Enable', 'on');
set(handles.BTN_compare2, 'Enable', 'on');
set(handles.BTN_rand_initial, 'Enable', 'on');
set(handles.BTN_rand_final, 'Enable', 'on');



function BTN_plan_Callback(hObject, eventdata, handles)
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

para.printing = true;
% method = 'pickplace';
method = 'pivoting';
% load debug
[path_found, plan] = solveAProblem(q0, qf, qg0, qgf, grasp_id_0, grasp_id_f, method);


if path_found
	set(handles.BTN_animate, 'Enable', 'on');
else
	set(handles.BTN_animate, 'Enable', 'off');
end


function BTN_rand_initial_Callback(hObject, eventdata, handles)
global mesh q0
q0 = quatRand();
plotObject(mesh, handles.AX_initial, q0);


function BTN_rand_final_Callback(hObject, eventdata, handles)
global mesh qf
qf = quatRand();
plotObject(mesh, handles.AX_final, qf);


function BTN_animate_Callback(hObject, eventdata, handles)
global path_found plan

if path_found
	animatePlan(5, plan);
end


% --- Executes on selection change in LB_files.
function LB_files_Callback(hObject, eventdata, handles)
global filename;
% hObject    handle to LB_files (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns LB_files contents as cell array
%        contents{get(hObject,'Value')} returns selected item from LB_files
contents = cellstr(get(hObject,'String'));
filename = contents{get(hObject,'Value')};


% --- Executes during object creation, after setting all properties.
function LB_files_CreateFcn(hObject, eventdata, handles)
% hObject    handle to LB_files (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in BTN_rand_grasp0.
function BTN_rand_grasp0_Callback(hObject, eventdata, handles)
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
function BTN_rand_graspf_Callback(hObject, eventdata, handles)
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
function CB_auto_grasp0_Callback(hObject, eventdata, handles)
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
	gripper_cone_width = getTiltedGripperCone(grasp_id, q0, para.GRIPPER_TILT_LIMIT);
	qg0                = getProperGrasp(grasp_id, q0, gripper_cone_width, false);
	
	plotObject(mesh, handles.AX_initial, q0);
	plotGripper(handles.AX_initial, gripper, q0, [gp10_w gp20_w], qg0);
	rotate3d on;
end

% --- Executes on button press in CB_auto_graspf.
function CB_auto_graspf_Callback(hObject, eventdata, handles)
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
	gripper_cone_width = getTiltedGripperCone(grasp_id, qf, para.GRIPPER_TILT_LIMIT);
	qgf                = getProperGrasp(grasp_id, qf, gripper_cone_width, false);

	plotObject(mesh, handles.AX_final, qf);
	plotGripper(handles.AX_final, gripper, qf, [gp1f_w gp2f_w], qgf);
	rotate3d on;
end

% --- Executes on slider movement.
function Slider_grasp0_Callback(hObject, eventdata, handles)
% hObject    handle to Slider_grasp0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Slider_grasp0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Slider_grasp0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function Slider_graspf_Callback(hObject, eventdata, handles)
% hObject    handle to Slider_graspf (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Slider_graspf_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Slider_graspf (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% 	Re-orienting test
function BTN_compare_Callback(hObject, eventdata, handles)
clc;
global para fgraph pgraph mesh gripper grasps % inputs

para.printing = false;



% ------------------------------------------
% 	get list of objects
% ------------------------------------------
files = dir('../model/data/*.mat');
Nobj  = length(files);

pivoting.plans       = cell(1, Nobj);
pivoting.scores      = cell(1, Nobj);
pivoting.path_found  = [];
pickplace.plans      = cell(1, Nobj);
pickplace.scores     = cell(1, Nobj);
pickplace.path_found = [];

for i_obj = 1:Nobj
	load(files(i_obj).name);
	disp(['[Object #' num2str(i_obj) '] ---- ' files(i_obj).name ' ----']);

	% get the list of stable poses
	% 	fgraph.quat(:,i)
	% 	fgraph.grasps(i, :)
	% 	fgraph.NM

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

		[pivoting_path_found(p), pivoting.plans{i_obj}{p}]   = solveAProblem(q0, qf, [], [], [], [], 'pivoting');
		[pickplace_path_found(p), pickplace.plans{i_obj}{p}] = solveAProblem(q0, qf, [], [], [], [], 'pickplace');

%         assert(length(pivoting.path_found) == NM);
		if pickplace_path_found(p) && ~pivoting_path_found(p)
			error('weird');
		end

		pivoting.scores{i_obj}{p}  = evalPlan(pivoting.plans{i_obj}{p});
		pickplace.scores{i_obj}{p} = evalPlan(pickplace.plans{i_obj}{p});

		disp(['		Problem #' num2str(p) ' of ' num2str(NP_i) ', Pivoting: ' num2str(pivoting_path_found(p)) ', P&P: ' num2str(pickplace_path_found(p))]);
	end

	pivoting.path_found  = [pivoting.path_found pivoting_path_found];
	pickplace.path_found = [pickplace.path_found pickplace_path_found];
end

save '../model/results/comparison1.mat' pivoting pickplace;
set(handles.BTN_show_results, 'Enable', 'on');





% --- Executes on button press in BTN_show_results.
function BTN_show_results_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_show_results (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
load 'comparison1.mat'
clc;

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
			pivoting_ex_time(count)      = pivoting.scores{i_obj}{i}.execution_time;
			pivoting_rotation(count)     = pivoting.scores{i_obj}{i}.gripper_rotation;        
			pivoting_translation(count)  = pivoting.scores{i_obj}{i}.gripper_translation;  
			pivoting_NRegrasp(count)     = pivoting.scores{i_obj}{i}.N_regrasp;
		end
		if pickplace.path_found(count)
			pickplace_ex_time(count)     = pickplace.scores{i_obj}{i}.execution_time;
			pickplace_rotation(count)    = pickplace.scores{i_obj}{i}.gripper_rotation;
			pickplace_translation(count) = pickplace.scores{i_obj}{i}.gripper_translation;  
			pickplace_NRegrasp(count)    = pickplace.scores{i_obj}{i}.N_regrasp;

			pivoting_ex_time_total      = pivoting_ex_time_total      + pivoting_ex_time(count);
			pivoting_rotation_total     = pivoting_rotation_total     + pivoting_rotation(count);
			pivoting_translation_total  = pivoting_translation_total  + pivoting_translation(count);
			pivoting_NRegrasp_total     = pivoting_NRegrasp_total     + pivoting_NRegrasp(count);
			pickplace_ex_time_total     = pickplace_ex_time_total     + pickplace_ex_time(count);
			pickplace_rotation_total    = pickplace_rotation_total    + pickplace_rotation(count);
			pickplace_translation_total = pickplace_translation_total + pickplace_translation(count);
			pickplace_NRegrasp_total    = pickplace_NRegrasp_total    + pickplace_NRegrasp(count);
		end
	end
end

pivoting_ex_time_aver      = pivoting_ex_time_total/N_pickplace_found;
pivoting_rotation_aver     = pivoting_rotation_total/N_pickplace_found;
pivoting_translation_aver  = pivoting_translation_total/N_pickplace_found;
pivoting_NRegrasp_aver     = pivoting_NRegrasp_total/N_pickplace_found;
pickplace_ex_time_aver     = pickplace_ex_time_total/N_pickplace_found;
pickplace_rotation_aver    = pickplace_rotation_total/N_pickplace_found;
pickplace_translation_aver = pickplace_translation_total/N_pickplace_found;
pickplace_NRegrasp_aver    = pickplace_NRegrasp_total/N_pickplace_found;

% disp(['EX time: ' num2str(sum(pivoting_ex_time_aver)) '		' num2str(sum(pickplace_ex_time_aver))] );
disp(['Total rotation: ' num2str(sum(pivoting_rotation_aver)) '		' num2str(sum(pickplace_rotation_aver))] );
disp(['Total translation: ' num2str(sum(pivoting_translation_aver)) '		' num2str(sum(pickplace_translation_aver))] );
disp(['# of regrasp: ' num2str(sum(pivoting_NRegrasp_aver)) '		' num2str(sum(pickplace_NRegrasp_aver))] );




% --- Executes on button press in BTN_compare2.
function BTN_compare2_Callback(hObject, eventdata, handles)
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
function BTN_show_results2_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_show_results2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
