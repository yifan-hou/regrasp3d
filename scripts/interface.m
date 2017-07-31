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

% Last Modified by GUIDE v2.5 31-Jul-2017 11:22:49

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

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes interface wait for user response (see UIRESUME)
% uiwait(handles.figure1);
addpath ../grasp
addpath ../model
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
global para fgraph pgraph mesh grasps q0 qf
% -----------------------------------------------
% 		Offline-computation
% -----------------------------------------------
para.GRIPPER_TILT_LIMIT = 40*pi/180; % tilting angle tolerance
para.GRIPPER_Z_LIMIT    = 0.2; % finger position limit
% friction between object and  ground
para.MU = 0.5;
% grasp pos sample density
para.GS_DENSITY = 0.1; % 1 point every 0.1 m^2
% grasp axis tolerance
para.ANGLE_TOL = 0.1; % rad
para.COM_DIST_LIMIT = 0.8; % meter
% shape of gripper(for collision checking)
% para.gripper_shape = getGripper();
para.GOALSAMPLEDENSITY2D = 15*pi/180; % 1 sample every 5 degree
para.PIVOTABLE_CHECK_GRANULARITY = 1*pi/180; % 1 sample every 1 degree

% Popups 
para.showObject             = false;
para.showObject_id          = 1;
para.showAllGraspSamples    = false;
para.showAllGraspSamples_id = 1;
para.showCheckedGrasp       = false;
para.showCheckedGrasp_id    = 3;
para.showProblem            = false;
para.showProblem_id         = [1 2 3];
para.show2Dproblem          = false;
para.show2Dproblem_id       = 4;

% get object mesh
[fgraph, pgraph, mesh] = getObject(para);

% calculate grasps, and contact mode graph
[grasps, fgraph] = calGrasp(fgraph, mesh, para);

% Plot the object
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
global para fgraph pgraph mesh grasps q0 qf % inputs
global path_found path_q path_graspid path_qp path_gripper_plan_2d % outputs

% get grasps for initial and final pose
grasp_id_0 = checkGrasp(grasps, mesh, q0, para);
grasp_id_f = checkGrasp(grasps, mesh, qf, para);

% treat initial/final pose as additional mode
% build the full graph
connect_matrix = zeros(fgraph.NM+2);
connect_matrix(1:fgraph.NM, 1:fgraph.NM) = fgraph.connect_matrix;
for m = 1:fgraph.NM
	if any(grasp_id_0&fgraph.grasps(m,:))
		connect_matrix(m, fgraph.NM+1) = 1;
		connect_matrix(fgraph.NM+1, m) = 1;
	end
	if any(grasp_id_f&fgraph.grasps(m,:))
		connect_matrix(m, fgraph.NM+2) = 1;
		connect_matrix(fgraph.NM+2, m) = 1;
	end
end
if any(grasp_id_f&grasp_id_0)
	connect_matrix(fgraph.NM+1, fgraph.NM+2) = 1;
	connect_matrix(fgraph.NM+2, fgraph.NM+1) = 1;
end
mode_grasps = [fgraph.grasps; grasp_id_0; grasp_id_f];

path_counter = 1;
while true
	[~, mode_id_path] = dijkstra(connect_matrix, ones(fgraph.NM+2), fgraph.NM+1, fgraph.NM+2);
	% mode_id_path = graph_search(connect_matrix, fgraph.NM+1, fgraph.NM+2);
	NP           = length(mode_id_path);
	if NP == 0
		disp('[Conclusion] No solution found given the available grasps.');
		break;
	end
	disp(['[Path ' num2str(path_counter) '] length = ' num2str(NP) ]);

	path_q               = zeros(4, NP);
	path_graspid         = zeros(1, NP-1);
	path_qp              = zeros(4, NP-1);
	path_gripper_plan_2d = cell(1,  NP-1);
	path_found           = false;

	% motion planning for each edge on the path
	path_q(:,1) = q0;
	for p = 1:NP-1
		if p == NP-1
			path_q(:,p+1) = qf;
		else
			path_qf(:,p+1) = fgraph.quat(:, mode_id_path(p+1));
		end
			
		id_common = find(mode_grasps(mode_id_path(p),:) == mode_grasps(mode_id_path(p+1),:));	
		assert(~isempty(id_common));
		disp(['# Edge ' num2str(p) ', common grasps: ' num2str(length(id_common))]);

		for i = 1:length(id_common)
			path_graspid(p) = id_common(i);
			gp1o_w          = grasps.points(:,id_common(i), 1);
			gp2o_w          = grasps.points(:,id_common(i), 2);

			[griper_plan_temp, qp_temp] = planOneGrasp(mesh, gp1o_w, gp2o_w, path_q(:,p), path_q(:,p+1), pgraph, para);

		    if isempty(griper_plan_temp)
		        disp([' -- Grasp ' num2str(i) ', No solution']);
		        continue;
		    else
				path_gripper_plan_2d{p} = griper_plan_temp;
				path_qp(:,p)            = qp_temp;
		        disp([' -- Grasp ' num2str(i) ' works.']);
		        disp(path_gripper_plan_2d{p});
		        break;
		    end
			% gripper motion closed loop control
		end

		if isempty(path_gripper_plan_2d{p})
			disp(['# Edge ' num2str(p) ', No solution.']);
			connect_matrix(mode_id_path(p), mode_id_path(p+1)) = 0;
			connect_matrix(mode_id_path(p+1), mode_id_path(p)) = 0;
			break;
		elseif p == NP-1
			path_found = true;
		end

	end % end a path

	if path_found
		disp('[Conclusion] Solution found. ');
% 		disp(path_found);
		break;
	end

end % end graph search

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
global para mesh grasps
global path_found path_q path_graspid path_qp path_gripper_plan_2d

if path_found
	animatePlan(mesh, grasps, handles.AX_animation, path_q, path_graspid, path_qp, path_gripper_plan_2d);
end
