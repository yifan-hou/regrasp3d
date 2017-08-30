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

% Last Modified by GUIDE v2.5 30-Aug-2017 11:00:06

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
addpath ../grasp
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
% -----------------------------------------------
% 		Offline-computation
% -----------------------------------------------
% Constraints
para.GRIPPER_TILT_LIMIT = 40*pi/180; % tilting angle tolerance
para.GRIPPER_Z_LIMIT    = 0.2; % finger position limit


% friction between object and  ground
para.MU = 0.5;

% Grasp sampling
para.NGS            = 200; % number of grasp pos samplings
para.ANGLE_TOL      = 0.1; % rad  % grasp axis tolerance
para.COM_DIST_LIMIT = 0.8; % meter

% planning parameter
para.PIVOTABLE_CHECK_GRANULARITY = 1*pi/180; % 1 sample every 1 degree
para.COLLISION_FREE_ANGLE_MARGIN = 5; % stay away from collsion for at least 5 degrees
									  % has to be an positive integer

% Popups 
para.showObject             = true; % show object and the simplified object
para.showObject_id          = [1 2 3];
para.showStablePoses        = false;
para.showStablePoses_id     = 1;
para.showCheckedGrasp       = true;
para.showCheckedGrasp_id    = 1;
para.showProblem            = false;
para.showProblem_id         = [1 2 3];
para.show2Dproblem          = false;
para.show2Dproblem_id       = 4;

% get object mesh
% filename = 'planefrontstay.stl';

% [fgraph, pgraph, mesh, mesh_s] = getObject(para, filename);
gripper                        = getGripper();
% [grasps, fgraph]               = calGrasp(fgraph, pgraph, mesh, mesh_s, gripper, para);
% save ../model/data/planefrontstay_data.mat fgraph pgraph mesh mesh_s grasps fgraph;

load(filename);


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
global para fgraph pgraph mesh gripper grasps q0 qf % inputs
global path_found path_q path_graspid path_qp plan_2d % outputs

disp('[Planning] Planning begin.');
% get grasp points for initial and final pose
grasp_id_0 = checkGraspPoints(grasps, mesh, pgraph, q0, para);
grasp_id_f = checkGraspPoints(grasps, mesh, pgraph, qf, para);

% treat initial/final pose as additional mode
% build the full graph
disp('[Planning] Building Full Graph:');
adj_matrix = zeros(fgraph.NM+2);
adj_matrix(1:fgraph.NM, 1:fgraph.NM) = fgraph.adj_matrix;
for m = 1:fgraph.NM
	if any(grasp_id_0&fgraph.grasps(m,:))
		adj_matrix(m, fgraph.NM+1) = 1;
		adj_matrix(fgraph.NM+1, m) = 1;
	end
	if any(grasp_id_f&fgraph.grasps(m,:))
		adj_matrix(m, fgraph.NM+2) = 1;
		adj_matrix(fgraph.NM+2, m) = 1;
	end
end
adj_matrix(fgraph.NM+1, fgraph.NM+1) = 1;
adj_matrix(fgraph.NM+2, fgraph.NM+2) = 1;
disp('Adjacent Matrix:');
disp(adj_matrix);
if any(grasp_id_f&grasp_id_0)
	adj_matrix(fgraph.NM+1, fgraph.NM+2) = 1;
	adj_matrix(fgraph.NM+2, fgraph.NM+1) = 1;
end
if ~any(grasp_id_0)
	set(handles.BTN_animate, 'Enable', 'off');
	disp('[Planning] No Solution. No feasible initial grasps.');
	return;
end
if ~any(grasp_id_f)
	set(handles.BTN_animate, 'Enable', 'off');
	disp('[Planning] No Solution. No feasible final grasps.');
	return;
end

mode_grasps = [fgraph.grasps; grasp_id_0; grasp_id_f];
path_counter = 1;
while true
	[~, mode_id_path] = dijkstra(adj_matrix, ones(fgraph.NM+2), fgraph.NM+1, fgraph.NM+2);
	NP                = length(mode_id_path);
	if isnan(mode_id_path)
		disp('[Planning] No solution found given the available grasps.');
		break;
	end
	disp(['[Planning] Trying Path #' num2str(path_counter) ', Path length = ' num2str(NP) ]);
	disp('[Planning] Planning for each edge on the path: ');

	path_q       = zeros(4, NP);
	path_graspid = zeros(1, NP-1);
	path_qp      = zeros(4, NP-1);
	plan_2d      = cell(1,  NP-1);
	path_found   = false;

	% motion planning for each edge on the path
	path_q(:,1) = q0;
	for p = 1:NP-1
		if p == NP-1
			path_q(:,p+1) = qf;
		else
			path_q(:,p+1) = fgraph.quat(:, mode_id_path(p+1));
		end
			
		id_common = find(mode_grasps(mode_id_path(p),:) == mode_grasps(mode_id_path(p+1),:));	
		assert(~isempty(id_common));
		disp(['  Edge #' num2str(p) ', common grasps: ' num2str(length(id_common))]);
		for i = 1:length(id_common)
			path_graspid(p) = id_common(i);

			[plan_2d_temp, qp_temp, flag] = planOneGrasp(mesh, grasps, id_common(i), path_q(:,p), path_q(:,p+1), pgraph, para);

		    if flag <= 0
                switch flag
                    case -1
                        disp(['  --- Grasp ' num2str(i) ' Required rolling exceeds gripper tilt limit']);
                    case -3
                        disp(['  --- Grasp ' num2str(i) ' Initial/final grasp pos violates gripper tilt limit ']);
                    case -4
                        disp(['  --- Grasp ' num2str(i) ' Initial/final grasp pos violates gripper Z limit ']);
                    case -5
                        disp(['  --- Grasp ' num2str(i) ' Gripper motion infeasible ']);
                    otherwise
                        error('Wrong flag');
                end
		        continue;
		    else
				plan_2d{p} = plan_2d_temp;
				path_qp(:,p)            = qp_temp;
		        disp(['  --- Grasp ' num2str(i) ' works.']);
		        break;
		    end
			% gripper motion closed loop control
		end

		if isempty(plan_2d{p})
			disp(['  Edge #' num2str(p) ' No solution.']);
			adj_matrix(mode_id_path(p), mode_id_path(p+1)) = 0;
			adj_matrix(mode_id_path(p+1), mode_id_path(p)) = 0;
			break;
		elseif p == NP-1
			path_found = true;
		end

	end % end a path

	if path_found
		disp(['[Planning] Solution found. Length = ' num2str(NP) ]);
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
global para mesh grasps gripper
global path_found path_q path_graspid path_qp plan_2d

if path_found
	animatePlan(mesh, grasps, gripper, handles.AX_animation, path_q, path_graspid, path_qp, plan_2d);
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
