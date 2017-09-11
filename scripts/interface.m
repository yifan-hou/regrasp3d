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

% Last Modified by GUIDE v2.5 07-Sep-2017 16:51:41

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

% ploting control
para.showCheckedGrasp     = true;
para.showCheckedGrasp_id  = 1;
para.showGraspChecking    = false;
para.showGraspChecking_id = [2 3];
para.showProblem          = false;
para.showProblem_id       = [1 2 3];
para.show2Dproblem        = false;
para.show2Dproblem_id     = 4;


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
rotate3d on;

q0 = [1 0 0 0]';
qf = [1 0 0 0]';

disp('[Load Model] Model is loaded.');

set(handles.BTN_plan, 'Enable', 'on');
set(handles.BTN_rand_initial, 'Enable', 'on');
set(handles.BTN_rand_final, 'Enable', 'on');



function BTN_plan_Callback(hObject, eventdata, handles)
clc;
global para fgraph pgraph mesh gripper grasps q0 qf qg0 qgf% inputs
global path_found plan_3d % outputs
global grasp_id_0 grasp_id_f

disp('[Planning] Planning begin.');

% get grasp points/grasp angles for initial and final pose
if get(handles.CB_auto_grasp0, 'Value')
	grasp_id_0 = checkGraspPoints(grasps, mesh, pgraph, q0, para.showGraspChecking_id(1), para);
	qg0 = [];
end

if get(handles.CB_auto_graspf, 'Value')
	grasp_id_f = checkGraspPoints(grasps, mesh, pgraph, qf, para.showGraspChecking_id(2), para);
	qgf = [];
end


[path_found, plan_3d] = solveAProblem(q0, qf, qg0, qgf, grasp_id_0, grasp_id_f);


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
global path_found plan_3d

if path_found
	animatePlan(handles.AX_animation, plan_3d);
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
global q0 qg0 grasp_id_0
global grasps gripper mesh pgraph para
% hObject    handle to BTN_rand_grasp0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% check all grasps for current pose
all_grasps = checkGraspPoints(grasps, mesh, pgraph, q0, para.showGraspChecking_id(2), para);
ids        = find(all_grasps);
for i = 1:length(ids)
	gp1o_w   = grasps.points(:, ids(i), 1);
	gp2o_w   = grasps.points(:, ids(i), 2);
	gp10_w   = quatOnVec(gp1o_w, q0);
	gp20_w   = quatOnVec(gp2o_w, q0);
	qg0      = getProperGrasp(gp10_w, gp20_w, grasps.range(:, ids(i)), q0, gp1o_w, gp2o_w, para); % grasp frame for q0, under world coordinate
	if(angBTVec([0 0 1]', quatOnVec([0 0 1]', qg0)) > para.GRIPPER_TILT_LIMIT)
		all_grasps(ids(i)) = false;
	end
end	

% pick one
ids                 = find(all_grasps);
if isempty(ids)
	disp('No feasible grasps for this pose !!');
	return;
end
id                  = randi(length(ids));
grasp_id_0          = zeros(size(all_grasps));
grasp_id_0(ids(id)) = 1;

% re-draw the object and gripper
grasp_id = find(grasp_id_0);
gp1o_w   = grasps.points(:, grasp_id, 1);
gp2o_w   = grasps.points(:, grasp_id, 2);
gp10_w   = quatOnVec(gp1o_w, q0);
gp20_w   = quatOnVec(gp2o_w, q0);
qg0      = getProperGrasp(gp10_w, gp20_w, grasps.range(:, grasp_id), q0, gp1o_w, gp2o_w, para); % grasp frame for q0, under world coordinate

plotObject(mesh, handles.AX_initial, q0);
plotGripper(handles.AX_initial, gripper, q0, [gp10_w gp20_w], qg0);
rotate3d on;


% --- Executes on button press in BTN_rand_graspf.
function BTN_rand_graspf_Callback(hObject, eventdata, handles)
global qf qgf grasp_id_f
global grasps gripper mesh pgraph para
% hObject    handle to BTN_rand_grasp0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% check all grasps for current pose
all_grasps = checkGraspPoints(grasps, mesh, pgraph, qf, para.showGraspChecking_id(2), para);
ids        = find(all_grasps);
for i = 1:length(ids)
	gp1o_w   = grasps.points(:, ids(i), 1);
	gp2o_w   = grasps.points(:, ids(i), 2);
	gp10_w   = quatOnVec(gp1o_w, qf);
	gp20_w   = quatOnVec(gp2o_w, qf);
	qgf      = getProperGrasp(gp10_w, gp20_w, grasps.range(:, ids(i)), qf, gp1o_w, gp2o_w, para); % grasp frame for qf, under world coordinate
	if(angBTVec([0 0 1]', quatOnVec([0 0 1]', qgf)) > para.GRIPPER_TILT_LIMIT)
		all_grasps(ids(i)) = false;
	end
end	

% pick one
ids                 = find(all_grasps);
if isempty(ids)
	disp('No feasible grasps for this pose !!');
	return;
end

id                  = randi(length(ids));
grasp_id_f          = zeros(size(all_grasps));
grasp_id_f(ids(id)) = 1;

% re-draw the object and gripper
grasp_id = find(grasp_id_f);
gp1o_w   = grasps.points(:, grasp_id, 1);
gp2o_w   = grasps.points(:, grasp_id, 2);
gp1f_w   = quatOnVec(gp1o_w, qf);
gp2f_w   = quatOnVec(gp2o_w, qf);
qgf      = getProperGrasp(gp1f_w, gp2f_w, grasps.range(:, grasp_id), qf, gp1o_w, gp2o_w, para); % grasp frame for qf, under world coordinate

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
	grasp_id = find(grasp_id_0);
	gp1o_w   = grasps.points(:, grasp_id, 1);
	gp2o_w   = grasps.points(:, grasp_id, 2);
	gp10_w   = quatOnVec(gp1o_w, q0);
	gp20_w   = quatOnVec(gp2o_w, q0);
	qg0      = getProperGrasp(gp10_w, gp20_w, grasps.range(:, grasp_id), q0, gp1o_w, gp2o_w, para); % grasp frame for q0, under world coordinate
	
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
	grasp_id = find(grasp_id_f);
	gp1o_w   = grasps.points(:, grasp_id, 1);
	gp2o_w   = grasps.points(:, grasp_id, 2);
	gp1f_w   = quatOnVec(gp1o_w, qf);
	gp2f_w   = quatOnVec(gp2o_w, qf);
	qgf      = getProperGrasp(gp1f_w, gp2f_w, grasps.range(:, grasp_id), qf, gp1o_w, gp2o_w, para); % grasp frame for q0, under world coordinate

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


% --- Executes on button press in BTN_compare.
function BTN_compare_Callback(hObject, eventdata, handles)
clc;
global para fgraph pgraph mesh gripper grasps q0 qf qg0 qgf% inputs
global path_found  plan_3d % outputs
global grasp_id_0 grasp_id_f

% ------------------------------------------
% 	Re-orienting test
% ------------------------------------------
% get list of objects

% get the list of stable poses
for i = 1:fgraph.NM
	qobj = fgraph.quat(:,i);
	grasp_id = fgraph.grasps(i, :);
end

% get list of problems


% solve by pivoting-regrasping
% [path_found, path_q, path_graspid, path_qp, plan_2d] = solveAProblem(q0, qf, qg0, qgf, grasp_id_0, grasp_id_f);
% solve by pick-and-place-regrasping

% measure:
% 	solvable
% 	length of robot motions
% 	number of regrasps




% % get grasp points/grasp angles for initial and final pose
% if get(handles.CB_auto_grasp0, 'Value')
% 	grasp_id_0 = checkGraspPoints(grasps, mesh, pgraph, q0, para.showGraspChecking_id(1), para);
% 	qg0 = [];
% end

% if get(handles.CB_auto_graspf, 'Value')
% 	grasp_id_f = checkGraspPoints(grasps, mesh, pgraph, qf, para.showGraspChecking_id(2), para);
% 	qgf = [];
% end


% [path_found, path_q, path_graspid, path_qp, plan_2d] = solveAProblem(q0, qf, qg0, qgf, grasp_id_0, grasp_id_f);


% if path_found
% 	set(handles.BTN_animate, 'Enable', 'on');
% else
% 	set(handles.BTN_animate, 'Enable', 'off');
% end

