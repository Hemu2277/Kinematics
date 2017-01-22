function varargout = GUI_4RP(varargin)
% GUI_4RP MATLAB code for GUI_4RP.fig
%      GUI_4RP, by itself, creates a new GUI_4RP or raises the existing
%      singleton*.
%
%      H = GUI_4RP returns the handle to a new GUI_4RP or the handle to
%      the existing singleton*.
%
%      GUI_4RP('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_4RP.M with the given input arguments.
%
%      GUI_4RP('Property','Value',...) creates a new GUI_4RP or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUI_4RP_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_4RP_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI_4RP

% Last Modified by GUIDE v2.5 15-Dec-2015 18:09:18

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_4RP_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_4RP_OutputFcn, ...
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


% --- Executes just before GUI_4RP is made visible.
function GUI_4RP_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUI_4RP (see VARARGIN)

% Choose default command line output for GUI_4RP
handles.output = hObject;

axes(handles.axes1)
cla()
view(2)
axes(handles.axes2)
cla()
box on
axis manual
axis equal
axis([-2 4 -2 4])

% --- User specified values:
% Set the frame offsets
handles.rO1 = [0 0];
handles.rO2 = [2 0];
handles.rO3 = [0 2];
handles.rO4 = [2 2];

set(handles.ro1_x,'String',num2str(handles.rO1(1)))
set(handles.ro1_y,'String',num2str(handles.rO1(2)))
set(handles.ro2_x,'String',num2str(handles.rO2(1)))
set(handles.ro2_y,'String',num2str(handles.rO2(2)))
set(handles.ro3_x,'String',num2str(handles.rO3(1)))
set(handles.ro3_y,'String',num2str(handles.rO3(2)))
set(handles.ro4_x,'String',num2str(handles.rO4(1)))
set(handles.ro4_y,'String',num2str(handles.rO4(2)))


% Set the starting end_affector position.
handles.x = 1;
handles.y = 1;
handles.D = zeros(4,2);

% calculate the starting values for all the arms depending on the initial configuration
[handles.d1, handles.theta1] = RPR3_InverseKinetics2(handles.x, handles.y, handles.rO1);
[handles.d2, handles.theta2] = RPR3_InverseKinetics2(handles.x, handles.y, handles.rO2);
[handles.d3, handles.theta3] = RPR3_InverseKinetics2(handles.x, handles.y, handles.rO3);
[handles.d4, handles.theta4] = RPR3_InverseKinetics2(handles.x, handles.y, handles.rO4);
set(handles.D1,'String',num2str(handles.d1))

% Set maximum displacements for att the arms
handles.d1max = 3;
handles.d2max = 3;
handles.d3max = 3;
handles.d4max = 3;

set(handles.d1max_edit,'String',num2str(handles.d1max))
set(handles.d2max_edit,'String',num2str(handles.d2max))
set(handles.d3max_edit,'String',num2str(handles.d3max))
set(handles.d4max_edit,'String',num2str(handles.d4max))


%Default leg is 1.
% handles.leg = 1;

% Initilize the plots
axis equal
hold on
box on
% axis([0 2 0 2])
handles.harm1 = plot([0 0],[0 0],'b','LineWidth',4);
handles.rO1_circle = plot(handles.rO1(1), handles.rO1(2), '.b', 'MarkerSize',30);

handles.harm2 = plot([0 0],[0 0],'r','LineWidth',4);
handles.rO2_circle = plot(handles.rO2(1), handles.rO2(2), '.r', 'MarkerSize',30);

handles.harm3 = plot([0 0],[0 0],'g','LineWidth',4);
handles.rO3_circle = plot(handles.rO3(1), handles.rO3(2), '.g', 'MarkerSize',30);

handles.harm4 = plot([0 0],[0 0],'k','LineWidth',4);
handles.rO4_circle = plot(handles.rO4(1), handles.rO4(2), '.k', 'MarkerSize',30);

handles.end_affector = plot(handles.x,handles.y,'ko','LineWidth',2);

handles.a = 1;
handles.b = 1;
handles.major_axis = 0.5;
handles.minor_axis = 0.5;
handles.x0 = 0;
handles.y0 = 0;
set(handles.a_edit,'String',num2str(handles.a))
set(handles.b_edit,'String',num2str(handles.b))
set(handles.major_axis_edit,'String',num2str(handles.major_axis))
set(handles.minor_axis_edit,'String',num2str(handles.minor_axis))
handles.circle_plot = plot(0,0);
handles.phi_dot = 2*pi;
set(handles.phi_dot_edit,'String',num2str(handles.phi_dot))

set(handles.choose_act_group,'selectedobject',[handles.J1_act_button handles.d1_act_button]);
handles.OldActive = handles.d1_act_button;
handles.NewActive = handles.J1_act_button;
handles.i_dot = sprintf('%s_dot',get(handles.NewActive,'String'));
handles.j_dot = sprintf('%s_dot',get(handles.OldActive,'String'));
handles.phi = linspace(0,360,100);

handles.k_gain = 2;
set(handles.control_gain_edit,'String',num2str(handles.k_gain))

handles.mag = 0.1;
handles.elements = 21;
set(handles.mag_edit,'String',num2str(handles.mag))
set(handles.elements_edit,'String',num2str(handles.elements))

axes(handles.axes1)
box on
axis equal
axis([-2 4 -2 4])

axes(handles.axes2)

message_out(hObject, eventdata, handles,'')


% Update handles structure and update plot
update_figure(hObject, eventdata, handles)
guidata(hObject, handles);

% UIWAIT makes GUI_4RP wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUI_4RP_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in update_button.
function update_button_Callback(hObject, eventdata, handles)
% hObject    handle to update_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
update_figure(hObject, eventdata, handles)


function x_edit_Callback(hObject, eventdata, handles)
% hObject    handle to x_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x_edit as text
%        str2double(get(hObject,'String')) returns contents of x_edit as a double
handles.x = str2double(get(hObject,'String'));

handles.D = [0 0 ;1 0;0 0;0 0]
guidata(hObject, handles);
update_values_inverse(hObject, eventdata, handles)
handles = guidata(hObject); % Needed to get the most recent handles changed in update_values_forward()
update_figure(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function x_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function y_edit_Callback(hObject, eventdata, handles)
% hObject    handle to y_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y_edit as text
%        str2double(get(hObject,'String')) returns contents of y_edit as a double
handles.y = str2double(get(hObject,'String'))
guidata(hObject, handles);
update_values_inverse(hObject, eventdata, handles)
handles = guidata(hObject); % Needed to get the most recent handles changed in update_values_forward()
update_figure(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function y_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function leg_edit_Callback(hObject, eventdata, handles)
% hObject    handle to leg_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of leg_edit as text
%        str2double(get(hObject,'String')) returns contents of leg_edit as a double
handles.leg = str2double(get(hObject,'String'));
guidata(hObject, handles);
update_figure(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function leg_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to leg_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta_edit_Callback(hObject, eventdata, handles)
% hObject    handle to theta_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta_edit as text
%        str2double(get(hObject,'String')) returns contents of theta_edit as a double
if handles.leg == 1
    handles.theta1 = str2double(get(hObject,'String'));
elseif handles.leg == 2
    handles.theta2 = str2double(get(hObject,'String'));
elseif handles.leg == 3
    handles.theta3 = str2double(get(hObject,'String'));
elseif handles.leg == 4
    handles.theta4 = str2double(get(hObject,'String'));
end

guidata(hObject, handles);
update_values_forward(hObject, eventdata, handles)
handles = guidata(hObject); % Needed to get the most recent handles changed in update_values_forward()
update_figure(hObject, eventdata, handles)
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function theta_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function d_edit_Callback(hObject, eventdata, handles)
% hObject    handle to d_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of d_edit as text
%        str2double(get(hObject,'String')) returns contents of d_edit as a double
if handles.leg == 1
    handles.d1 = str2double(get(hObject,'String'));
elseif handles.leg == 2
    handles.d2 = str2double(get(hObject,'String'));
elseif handles.leg == 3
    handles.d3 = str2double(get(hObject,'String'));
elseif handles.leg == 4
    handles.d4 = str2double(get(hObject,'String'));
end

guidata(hObject, handles);
update_values_forward(hObject, eventdata, handles)
handles = guidata(hObject); % Needed to get the most recent handles changed in update_values_forward()
update_figure(hObject, eventdata, handles)
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function d_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to d_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function update_figure(hObject, eventdata, handles)
%     axes(handles.axes2);
    axis([-2 4 -2 4])
%     disp(handles.rO1(1))
    set_strings(hObject, eventdata, handles)
    % Arm 1
    set(handles.harm1,'XData',handles.rO1(1) + [-(handles.d1max-handles.d1)*cosd(handles.theta1) handles.d1*cosd(handles.theta1)],...
                      'YData',handles.rO1(2) + [-(handles.d1max-handles.d1)*sind(handles.theta1) handles.d1*sind(handles.theta1)]);

    % Arm 2
    set(handles.harm2,'XData',handles.rO2(1) + [-(handles.d2max-handles.d2)*cosd(handles.theta2) handles.d2*cosd(handles.theta2)],...
                      'YData',handles.rO2(2) + [-(handles.d2max-handles.d2)*sind(handles.theta2) handles.d2*sind(handles.theta2)]);

    % Arm 3
    set(handles.harm3,'XData',handles.rO3(1) + [-(handles.d3max-handles.d3)*cosd(handles.theta3) handles.d3*cosd(handles.theta3)],...
                      'YData',handles.rO3(2) + [-(handles.d3max-handles.d3)*sind(handles.theta3) handles.d3*sind(handles.theta3)]);
    
    % Arm4              
    set(handles.harm4,'XData',handles.rO4(1) + [-(handles.d4max-handles.d4)*cosd(handles.theta4) handles.d4*cosd(handles.theta4)],...
                      'YData',handles.rO4(2) + [-(handles.d4max-handles.d4)*sind(handles.theta4) handles.d4*sind(handles.theta4)]);
   
    set(handles.end_affector,'XData',handles.x,'YData',handles.y)
    guidata(hObject, handles);

    
% function update_values_forward(hObject, eventdata, handles)
%     %Hemanth is doing
%     if handles.leg == 1
%         [handles.x, handles.y] = RPR3_ForwardKinetics2(handles.theta1,handles.d1, handles.rO1)
%         [handles.d2, handles.theta2] = RPR3_InverseKinetics2(handles.x, handles.y, handles.rO2)
%         [handles.d3, handles.theta3] = RPR3_InverseKinetics2(handles.x, handles.y, handles.rO3)
%         [handles.d4, handles.theta4] = RPR3_InverseKinetics2(handles.x, handles.y, handles.rO4)
%     elseif handles.leg == 2
%         [handles.x, handles.y] = RPR3_ForwardKinetics2(handles.theta2,handles.d2, handles.rO2)
%         [handles.d1, handles.theta1] = RPR3_InverseKinetics2(handles.x, handles.y, handles.rO1)
%         [handles.d3, handles.theta3] = RPR3_InverseKinetics2(handles.x, handles.y, handles.rO3)
%         [handles.d4, handles.theta4] = RPR3_InverseKinetics2(handles.x, handles.y, handles.rO4)
%     elseif handles.leg == 3
%         [handles.x, handles.y] = RPR3_ForwardKinetics2(handles.theta3,handles.d3, handles.rO3)
%         [handles.d1, handles.theta1] = RPR3_InverseKinetics2(handles.x, handles.y, handles.rO1)
%         [handles.d2, handles.theta2] = RPR3_InverseKinetics2(handles.x, handles.y, handles.rO2)
%         [handles.d4, handles.theta4] = RPR3_InverseKinetics2(handles.x, handles.y, handles.rO4)
%     elseif handles.leg == 4
%         [handles.x, handles.y] = RPR3_ForwardKinetics2(handles.theta4,handles.d4,handles.rO4)
%         [handles.d1, handles.theta1] = RPR3_InverseKinetics2(handles.x, handles.y, handles.rO1)
%         [handles.d2, handles.theta2] = RPR3_InverseKinetics2(handles.x, handles.y, handles.rO2)
%         [handles.d3, handles.theta3] = RPR3_InverseKinetics2(handles.x, handles.y, handles.rO3)
%     end
%     set_strings(hObject, eventdata, handles)
%     guidata(hObject, handles);

    
function update_values_inverse(hObject, eventdata, handles)
% Should check if the given end affecter position is in the manipulators
% workspace. If it is all the values are updates.
    [handles.d1, handles.theta1] = RPR3_InverseKinetics2(handles.x, handles.y, handles.rO1);
    [handles.d2, handles.theta2] = RPR3_InverseKinetics2(handles.x, handles.y, handles.rO2);
    [handles.d3, handles.theta3] = RPR3_InverseKinetics2(handles.x, handles.y, handles.rO3);
    [handles.d4, handles.theta4] = RPR3_InverseKinetics2(handles.x, handles.y, handles.rO4);
    
    guidata(hObject, handles);
% %     update_values_forward(hObject, eventdata, handles)
%     handles = guidata(hObject); % Needed to get the most recent handles changed in update_values_forward()
    update_figure(hObject, eventdata, handles)
    guidata(hObject, handles);
   

function set_strings(hObject, eventdata, handles)
    set(handles.x_edit,'String',sprintf('%2.2f',handles.x))
    set(handles.y_edit,'String',sprintf('%2.2f',handles.y))
    set(handles.D1,'String',num2str(handles.d1))
    set(handles.D2,'String',num2str(handles.d2))
    set(handles.D3,'String',num2str(handles.d3))
    set(handles.D4,'String',num2str(handles.d4))
    set(handles.theta_1,'String',num2str(handles.theta1))
    set(handles.theta_2,'String',num2str(handles.theta2))
    set(handles.theta_3,'String',num2str(handles.theta3))
    set(handles.theta_4,'String',num2str(handles.theta4))
    guidata(hObject, handles);

% function animateXY(hObject, eventdata, handles, x,y)
%     for i = 1:length(x)
%         handles.x = x(i);
%         handles.y = y(i);
%         guidata(hObject, handles);
%         update_values_inverse(hObject, eventdata, handles)
%         drawnow;
%     end
      



function D1_Callback(hObject, eventdata, handles)
% hObject    handle to D1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of D1 as text
%        str2double(get(hObject,'String')) returns contents of D1 as a double
handles.D(1,1) = str2double(get(hObject,'String'));
% disp(handles.D(1,1))
if sum(nnz(handles.D)) ==2
    [A,B,T] = config(handles.D);
    [handles.x,handles.y] = forward(A,B,T,handles.D);
    update_values_inverse(hObject, eventdata, handles)
    handles.D = zeros(4,2);
end
guidata(hObject, handles)



% --- Executes during object creation, after setting all properties.
function D1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to D1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function D2_Callback(hObject, eventdata, handles)
% hObject    handle to D2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of D2 as text
%        str2double(get(hObject,'String')) returns contents of D2 as a double
handles.D(2,1) = str2double(get(hObject,'String'));
% disp(handles.D(2,1))
if sum(nnz(handles.D)) ==2
    [A,B,T] = config(handles.D);
    [handles.x,handles.y] = forward(A,B,T,handles.D);
    update_values_inverse(hObject, eventdata, handles)
    handles.D = zeros(4,2);
end
guidata(hObject, handles)



% --- Executes during object creation, after setting all properties.
function D2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to D2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function D3_Callback(hObject, eventdata, handles)
% hObject    handle to D3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of D3 as text
%        str2double(get(hObject,'String')) returns contents of D3 as a double
handles.D(3,1) = str2double(get(hObject,'String'));
% disp(handles.D(3,1));
if sum(nnz(handles.D)) ==2
     [A,B,T] = config(handles.D);
    [handles.x,handles.y] = forward(A,B,T,handles.D);
    update_values_inverse(hObject, eventdata, handles)
    handles.D = zeros(4,2);
end
guidata(hObject, handles)



% --- Executes during object creation, after setting all properties.
function D3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to D3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function D4_Callback(hObject, eventdata, handles)
% hObject    handle to D4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of D4 as text
%        str2double(get(hObject,'String')) returns contents of D4 as a double
handles.D(4,1) = str2double(get(hObject,'String'));  
% disp(handles.D(4,1))
if sum(nnz(handles.D)) ==2
    [A,B,T] = config(handles.D);
    [handles.x,handles.y] = forward(A,B,T,handles.D);
    update_values_inverse(hObject, eventdata, handles)
    handles.D = zeros(4,2);
end
guidata(hObject, handles)



% --- Executes during object creation, after setting all properties.
function D4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to D4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta_1_Callback(hObject, eventdata, handles)
% hObject    handle to theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta_1 as text
%        str2double(get(hObject,'String')) returns contents of theta_1 as a double
handles.D(1,2) = str2double(get(hObject,'String'));
% disp(handles.D(1,2))
if sum(nnz(handles.D)) ==2
     [A,B,T] = config(handles.D);
    [handles.x,handles.y] = forward(A,B,T,handles.D);
    update_values_inverse(hObject, eventdata, handles)
    handles.D = zeros(4,2);
end
guidata(hObject, handles)


% --- Executes during object creation, after setting all properties.
function theta_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta_2_Callback(hObject, eventdata, handles)
% hObject    handle to theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta_2 as text
%        str2double(get(hObject,'String')) returns contents of theta_2 as a double
handles.D(2,2) = str2double(get(hObject,'String'));
% disp(handles.D(2,2))
if sum(nnz(handles.D)) ==2
     [A,B,T] = config(handles.D);
    [handles.x,handles.y] = forward(A,B,T,handles.D);
    update_values_inverse(hObject, eventdata, handles)
    handles.D = zeros(4,2);
end
guidata(hObject, handles)


% --- Executes during object creation, after setting all properties.
function theta_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta_3_Callback(hObject, eventdata, handles)
% hObject    handle to theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta_3 as text
%        str2double(get(hObject,'String')) returns contents of theta_3 as a double
handles.D(3,2) = str2double(get(hObject,'String'));
% disp(handles.D(3,2))
if sum(nnz(handles.D)) ==2
     [A,B,T] = config(handles.D);
    [handles.x,handles.y] = forward(A,B,T,handles.D);
    update_values_inverse(hObject, eventdata, handles)
    handles.D = zeros(4,2);
end
guidata(hObject, handles)


% --- Executes during object creation, after setting all properties.
function theta_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta_4_Callback(hObject, eventdata, handles)
% hObject    handle to theta_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta_4 as text
%        str2double(get(hObject,'String')) returns contents of theta_4 as a double
handles.D(4,2) = str2double(get(hObject,'String'));
% disp(handles.D(4,2))
if sum(nnz(handles.D)) ==2
     [A,B,T] = config(handles.D);
    [handles.x,handles.y] = forward(A,B,T,handles.D);
    update_values_inverse(hObject, eventdata, handles)
    handles.D = zeros(4,2);
end
guidata(hObject, handles)


% --- Executes during object creation, after setting all properties.
function theta_4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes during object creation, after setting all properties.
function space_CreateFcn(hObject, eventdata, handles)
% hObject    handle to space (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function initial_Callback(hObject, eventdata, handles)
% hObject    handle to initial (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of initial as text
%        str2double(get(hObject,'String')) returns contents of initial as a double
global xi yi 
XI = str2num(get(handles.initial,'String'));
xi = XI(:,1);
yi = XI(:,2);


% --- Executes during object creation, after setting all properties.
function initial_CreateFcn(hObject, eventdata, handles)
% hObject    handle to initial (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function final_Callback(hObject, eventdata, handles)
% hObject    handle to final (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of final as text
%        str2double(get(hObject,'String')) returns contents of final as a double
global xf yf
XF = str2num(get(handles.final,'String'));
xf = XF(:,1);
yf = XF(:,2);


% --- Executes during object creation, after setting all properties.
function final_CreateFcn(hObject, eventdata, handles)
% hObject    handle to final (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




% --- Executes during object creation, after setting all properties.
function sv_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sv (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


function space_Callback(hObject, eventdata, handles)
% hObject    handle to space (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns space contents as cell array
%        contents{get(hObject,'Value')} returns selected item from space
global str val
str = get(hObject,'String');
val = get(hObject, 'Value');




function vari_i_Callback(hObject, eventdata, handles)
% hObject    handle to vari_i (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of vari_i as text
%        str2double(get(hObject,'String')) returns contents of vari_i as a double
switch handles.str(handles.val)
    case 'Work Space'
       set(hObject, 'String','[xi,yi]');
    case 'Joint Space'
       set(hObject, 'String','theta_i');
end

        


% --- Executes during object creation, after setting all properties.
function vari_i_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vari_i (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function vari_f_Callback(hObject, eventdata, handles)
% hObject    handle to vari_f (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of vari_f as text
%        str2double(get(hObject,'String')) returns contents of vari_f as a double
switch (handles.str(handles.val))
    case 'Work_Space'
       set(handles.vari_f, 'String','[xf,yf]');
    case 'Joint_Space'
       set(handles.vari_f, 'String','theta_f');
end


% --- Executes during object creation, after setting all properties.
function vari_f_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vari_f (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on selection change in space.







% --- Executes on button press in linear.
function linear_Callback(hObject, eventdata, handles)
% hObject    handle to linear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of linear
global  inter_type
% Hint: get(hObject,'Value') returns toggle state of linear
inter_type = lower(get(handles.linear, 'String'));

% --- Executes on button press in cubic.
function cubic_Callback(hObject, eventdata, handles)
% hObject    handle to cubic (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cubic
global  inter_type
% Hint: get(hObject,'Value') returns toggle state of cubic
inter_type = lower(get(handles.cubic, 'String'));


% --- Executes on button press in quintic.
function quintic_Callback(hObject, eventdata, handles)
% hObject    handle to quintic (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of quintic
global  inter_type
% Hint: get(hObject,'Value') returns toggle state of quintic
inter_type = lower(get(handles.quintic, 'String'));



% --- Executes on button press in inter.
function inter_Callback(hObject, eventdata, handles)
% hObject    handle to inter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global xi yi xf yf inter_type
inter_type;
[x,y,t] = interpolate(xi,yi,xf,yf,10,inter_type);
axes(handles.axes1)
axis equal
box on
plot(t,x);
grid on 
hold on
plot(t,y);
legend('x','y')
hold off
guidata(hObject, handles);


function a_edit_Callback(hObject, eventdata, handles)
% hObject    handle to a_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of a_edit as text
%        str2double(get(hObject,'String')) returns contents of a_edit as a double

handles.a = str2double(get(hObject,'String'));
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function a_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to a_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function b_edit_Callback(hObject, eventdata, handles)
% hObject    handle to b_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of b_edit as text
%        str2double(get(hObject,'String')) returns contents of b_edit as a double
handles.b = str2double(get(hObject,'String'));
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function b_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to b_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function major_axis_edit_Callback(hObject, eventdata, handles)
% hObject    handle to major_axis_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of major_axis_edit as text
%        str2double(get(hObject,'String')) returns contents of major_axis_edit as a double
handles.major_axis = str2double(get(hObject,'String'));
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function major_axis_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to major_axis_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function minor_axis_edit_Callback(hObject, eventdata, handles)
% hObject    handle to minor_axis_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of minor_axis_edit as text
%        str2double(get(hObject,'String')) returns contents of minor_axis_edit as a double
handles.minor_axis = str2double(get(hObject,'String'));
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function minor_axis_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to minor_axis_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function phi_dot_edit_Callback(hObject, eventdata, handles)
% hObject    handle to phi_dot_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of phi_dot_edit as text
%        str2double(get(hObject,'String')) returns contents of phi_dot_edit as a double
handles.phi_dot = str2double(get(hObject,'String'));
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function phi_dot_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to phi_dot_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function choose_act_group_CreateFcn(hObject, eventdata, handles)
% hObject    handle to choose_act_group (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes when selected object is changed in choose_act_group.
function choose_act_group_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in choose_act_group 
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)
handles.OldActive = handles.NewActive
if handles.OldActive ~= eventdata.NewValue
    handles.NewActive = eventdata.NewValue;
    set(handles.choose_act_group,'selectedobject',[handles.OldActive handles.NewActive]);
end
fprintf('[%s  %s]''',get(handles.OldActive,'String'),get(handles.NewActive,'String'))
handles.i_dot = sprintf('%s_dot',get(handles.NewActive,'String'))
handles.j_dot = sprintf('%s_dot',get(handles.OldActive,'String'))
guidata(hObject, handles);




function getJacobian(hObject, eventdata, handles)
% calculates the jacobian dependant on the indipendant ariables supplied
% Saves the jacobian in handles.J that gives Xe_dot = J*[i_dot j_dot]'

% theta = [d1;      1
%          theta1;  2
%          d2;      3
%          theta2;  4
%          d3;      5
%          theta3;  6
%          d4;      7
%          theta4]  8

JA = [cosd(handles.theta1) -handles.d1*sind(handles.theta1);
      sind(handles.theta1)  handles.d1*cosd(handles.theta1)];
 
JB = [cosd(handles.theta2) -handles.d2*sind(handles.theta2);
      sind(handles.theta2)  handles.d2*cosd(handles.theta2)];
  
JC = [cosd(handles.theta3) -handles.d3*sind(handles.theta3);
      sind(handles.theta3)  handles.d3*cosd(handles.theta3)];
  
JD = [cosd(handles.theta4) -handles.d4*sind(handles.theta4);
      sind(handles.theta4)  handles.d4*cosd(handles.theta4)];
 
zeroM = zeros(2,2);
J = [JA -JB     zeroM   zeroM;
     JA zeroM   -JC     zeroM;
     JA zeroM   zeroM   -JD];
 
switch handles.i_dot 
    case 'd1_dot'
        iI(1) = 1;
    case 'theta1_dot'
        iI(1) = 2;
    case 'd2_dot'
        iI(1) = 3;
	case 'theta2_dot'
        iI(1) = 4;
    case 'd3_dot'
        iI(1) = 5;
	case 'theta3_dot'
        iI(1) = 6;
    case 'd4_dot'
        iI(1) = 7;
	case 'theta4_dot'
        iI(1) = 8;
end

switch handles.j_dot 
    case 'd1_dot'
        iI(2) = 1;
    case 'theta1_dot'
        iI(2) = 2;
    case 'd2_dot'
        iI(2) = 3;
	case 'theta2_dot'
        iI(2) = 4;
    case 'd3_dot'
        iI(2) = 5;
	case 'theta3_dot'
        iI(2) = 6;
    case 'd4_dot'
        iI(2) = 7;
	case 'theta4_dot'
        iI(2) = 8;
end

iD = 1:8;
iD = iD(iD ~= iI(1));
iD = iD(iD ~= iI(2));

JInd = J(:,iI);
JDep = J(:,iD);

Jm = -(JDep\JInd);

S = zeros(8,2);

S(iI(1),1) = 1;
S(iI(2),2) = 1;

k = 1;
for i = 1:8
    if i ~= iI(1)
        if i ~=iI(2)
            S(i,:) = Jm(k,:);
            k = k+1;
        end
    end
end
    
JAI = [S(1,:);
       S(2,:)];
% disp(JA*JAI)

JBI = [S(3,:);
       S(4,:)];
% disp(JB*JBI)

JCI = [S(5,:);
       S(6,:)];
% disp(JC*JCI)

JDI = [S(7,:);
       S(8,:)];
% disp(JD*JDI)

handles.J = JA*JAI; % use this as the jacobian in the work space analysis

% fprintf('rank = %f',rank(handles.J))
% if rank(handles.J) < 2
%     disp('Singularity detected in the jacobian')
% end

guidata(hObject, handles);


% --- Executes on button press in open_loop_run.
function open_loop_run_Callback(hObject, eventdata, handles)
% hObject    handle to open_loop_run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.control_type = 'Open_loop';
plot_path(hObject, eventdata, handles)
handles = guidata(hObject);
controls(hObject, eventdata, handles);
guidata(hObject, handles);


% --- Executes on button press in joint_space_run.
function joint_space_run_Callback(hObject, eventdata, handles)
% hObject    handle to joint_space_run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.control_type = 'Closed_loop_joint';
plot_path(hObject, eventdata, handles)
handles = guidata(hObject);
controls(hObject, eventdata, handles);
guidata(hObject, handles);

% --- Executes on button press in task_space_run.
function task_space_run_Callback(hObject, eventdata, handles)
% hObject    handle to task_space_run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.control_type = 'Closed_loop_task';
plot_path(hObject, eventdata, handles)
handles = guidata(hObject);
controls(hObject, eventdata, handles);
guidata(hObject, handles);

function ro1_x_Callback(hObject, eventdata, handles)
% hObject    handle to ro1_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ro1_x as text
%        str2double(get(hObject,'String')) returns contents of ro1_x as a double
handles.rO1(1) = str2double(get(hObject,'String'));
set(handles.rO1_circle,'XData',handles.rO1(1))
guidata(hObject, handles);
update_values_inverse(hObject, eventdata, handles)
handles = guidata(hObject);
update_figure(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function ro1_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ro1_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function ro1_y_Callback(hObject, eventdata, handles)
% hObject    handle to ro1_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ro1_y as text
%        str2double(get(hObject,'String')) returns contents of ro1_y as a double
handles.rO1(2) = str2double(get(hObject,'String'));
set(handles.rO1_circle,'YData',handles.rO1(2))
guidata(hObject, handles);
update_values_inverse(hObject, eventdata, handles)
handles = guidata(hObject);
update_figure(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function ro1_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ro1_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ro2_x_Callback(hObject, eventdata, handles)
% hObject    handle to ro2_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ro2_x as text
%        str2double(get(hObject,'String')) returns contents of ro2_x as a double
handles.rO2(1) = str2double(get(hObject,'String'))
set(handles.rO2_circle,'XData',handles.rO2(1))
guidata(hObject, handles);
update_values_inverse(hObject, eventdata, handles)
handles = guidata(hObject);

% --- Executes during object creation, after setting all properties.
function ro2_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ro2_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ro2_y_Callback(hObject, eventdata, handles)
% hObject    handle to ro2_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ro2_y as text
%        str2double(get(hObject,'String')) returns contents of ro2_y as a double
handles.rO2(2) = str2double(get(hObject,'String'));
set(handles.rO2_circle,'YData',handles.rO2(2))
guidata(hObject, handles);
update_values_inverse(hObject, eventdata, handles)
handles = guidata(hObject);
update_figure(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function ro2_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ro2_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ro3_x_Callback(hObject, eventdata, handles)
% hObject    handle to ro3_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ro3_x as text
%        str2double(get(hObject,'String')) returns contents of ro3_x as a double
handles.rO3(1) = str2double(get(hObject,'String'));
set(handles.rO3_circle,'XData',handles.rO3(1))
guidata(hObject, handles);
update_values_inverse(hObject, eventdata, handles)
handles = guidata(hObject);

% --- Executes during object creation, after setting all properties.
function ro3_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ro3_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function ro3_y_Callback(hObject, eventdata, handles)
% hObject    handle to ro3_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ro3_y as text
%        str2double(get(hObject,'String')) returns contents of ro3_y as a double
handles.rO3(2) = str2double(get(hObject,'String'));
set(handles.rO3_circle,'YData',handles.rO3(2))
guidata(hObject, handles);
update_values_inverse(hObject, eventdata, handles)
handles = guidata(hObject);
update_figure(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function ro3_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ro3_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ro4_x_Callback(hObject, eventdata, handles)
% hObject    handle to ro4_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ro4_x as text
%        str2double(get(hObject,'String')) returns contents of ro4_x as a double
handles.rO4(1) = str2double(get(hObject,'String'));
set(handles.rO4_circle,'XData',handles.rO4(1))
guidata(hObject, handles);
update_values_inverse(hObject, eventdata, handles)
handles = guidata(hObject);

% --- Executes during object creation, after setting all properties.
function ro4_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ro4_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ro4_y_Callback(hObject, eventdata, handles)
% hObject    handle to ro4_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ro4_y as text
%        str2double(get(hObject,'String')) returns contents of ro4_y as a double
handles.rO4(2) = str2double(get(hObject,'String'));
set(handles.rO4_circle,'YData',handles.rO4(2))
guidata(hObject, handles);
update_values_inverse(hObject, eventdata, handles)
handles = guidata(hObject);
update_figure(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function ro4_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ro4_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function d1max_edit_Callback(hObject, eventdata, handles)
% hObject    handle to d1max_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of d1max_edit as text
%        str2double(get(hObject,'String')) returns contents of d1max_edit as a double
handles.d1max = str2double(get(hObject,'String'));
guidata(hObject, handles);
update_figure(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function d1max_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to d1max_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function d2max_edit_Callback(hObject, eventdata, handles)
% hObject    handle to d2max_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of d2max_edit as text
%        str2double(get(hObject,'String')) returns contents of d2max_edit as a double
handles.d2max = str2double(get(hObject,'String'));
guidata(hObject, handles);
update_figure(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function d2max_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to d2max_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function d3max_edit_Callback(hObject, eventdata, handles)
% hObject    handle to d3max_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of d3max_edit as text
%        str2double(get(hObject,'String')) returns contents of d3max_edit as a double
handles.d3max = str2double(get(hObject,'String'));
guidata(hObject, handles);
update_figure(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function d3max_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to d3max_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function d4max_edit_Callback(hObject, eventdata, handles)
% hObject    handle to d4max_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of d4max_edit as text
%        str2double(get(hObject,'String')) returns contents of d4max_edit as a double
handles.d4max = str2double(get(hObject,'String'));
guidata(hObject, handles);
update_figure(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function d4max_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to d4max_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes when selected object is changed in uipanel7.
function uipanel7_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in uipanel7 
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)

% disp('her')
% dips('button pressed')
if (get(hObject,'Value') == get(hObject,'Max'))
    x = handles.a + handles.major_axis*cosd(handles.phi);
    y = handles.b + handles.minor_axis*sind(handles.phi);
    set(handles.circle_plot,'XData',x,'YData',y)
else
    set(handles.circle_plot,'XData',0,'YData',0)
end



% --- Executes on button press in plot_path.
function plot_path_Callback(hObject, eventdata, handles)
% hObject    handle to plot_path (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% disp('button pressed')
% if (get(hObject,'Value') == get(hObject,'Max'))
%     x = handles.a + handles.major_axis*cosd(handles.phi);
%     y = handles.b + handles.minor_axis*sind(handles.phi);
%     set(handles.circle_plot,'XData',x,'YData',y)
%     handles.x0 = x(1);
%     handles.y0 = y(1);
%     
% else
%     set(handles.circle_plot,'XData',0,'YData',0)
% end
plot_path(hObject, eventdata, handles)
guidata(hObject, handles)


% --------- This function should be implemeneted in future version------------
function out = check_reachable(hObject, eventdata, handles)
if (handles.x-handles.r01(1))^2 + (handles.y-handles.r01(2))^2 < handles.d1max^2
    if (handles.x-handles.r02(1))^2 + (handles.y-handles.r02(2))^2 < handles.d2max^2
        if (handles.x-handles.r03(1))^2 + (handles.y-handles.r03(2))^2 < handles.d3max^2
            if (handles.x-handles.r04(1))^2 + (handles.y-handles.r04(2))^2 < handles.d4max^2
                out = 1;
            end
        end
    end
end


% --- Executes during object creation, after setting all properties.
function message(hObject, eventdata, handles)
% hObject    handle to message_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function message_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to message_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in isotropy_button.
function isotropy_button_Callback(hObject, eventdata, handles)
% hObject    handle to isotropy_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
message_out(hObject, eventdata, handles,'Stand by - Generating Isotropy index')

xlim = get(gca,'Xlim');
ylim = get(gca,'Ylim');

x_start = handles.x;
y_start = handles.y;

elements = handles.elements;
x_d = linspace(xlim(1),xlim(2),elements);
y_d = linspace(ylim(1),ylim(2),elements);
[X,Y] = meshgrid(x_d,y_d);

for i = 1:elements
    for j = 1:elements
        handles.x = X(i,j);
        handles.y = Y(i,j);
        guidata(hObject, handles);

        update_values_inverse(hObject, eventdata, handles);
        handles = guidata(hObject);
        getJacobian(hObject, eventdata, handles);
        handles = guidata(hObject);

         % i) Calculate the Isotropy Measure of Manipulability
%          disp(handles.J)
        if any(isnan(handles.J))
            w(i,j) = 0;
        else
            [U,S,V] = svd(handles.J);
            w(i,j) = min(diag(S))/max(diag(S));
        end
    end
end
axes(handles.axes1);
cla()
axis normal
view(3)
mesh(X,Y,w)
axis([-2 4 -2 4 0 max(max(w))])

handles.x = x_start;
handles.y = y_start;
guidata(hObject, handles);

update_values_inverse(hObject, eventdata, handles);
message_out(hObject, eventdata, handles,'Done')
guidata(hObject, handles);

% --- Executes on button press in yoshikawa_button.
function yoshikawa_button_Callback(hObject, eventdata, handles)
% hObject    handle to yoshikawa_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
message_out(hObject, eventdata, handles,'Stand by - Generating Yoshikawa index')
xlim = get(gca,'Xlim');
ylim = get(gca,'Ylim');

x_start = handles.x;
y_start = handles.y;

elements = handles.elements;
x_d = linspace(xlim(1),xlim(2),elements);
y_d = linspace(ylim(1),ylim(2),elements);
[X,Y] = meshgrid(x_d,y_d);

for i = 1:elements
    for j = 1:elements
        handles.x = X(i,j);
        handles.y = Y(i,j);
        guidata(hObject, handles);

        update_values_inverse(hObject, eventdata, handles);
        handles = guidata(hObject);
        getJacobian(hObject, eventdata, handles);
        handles = guidata(hObject);

        % ii) 
%         disp(handles.J)
%         disp(sqrt(det(handles.J*handles.J')))
        w_yosh(i,j) = sqrt(det(handles.J*(handles.J')));
%                    w_yosh(i,j) = det(handles.J)

    end
end

axes(handles.axes1);
cla()
% axis manual
axis([-2 4 -2 4 0 max(max(w_yosh))])
view(3)
mesh(X,Y,w_yosh)
handles.x = x_start;
handles.y = y_start;
guidata(hObject, handles);

update_values_inverse(hObject, eventdata, handles);
message_out(hObject, eventdata, handles,'Done')
guidata(hObject, handles);

% --- Executes on button press in elipsoide_button.
function elipsoide_button_Callback(hObject, eventdata, handles)
% hObject    handle to elipsoide_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
message_out(hObject, eventdata, handles,'Stand by - Generating elipsoides')

xlim = get(gca,'Xlim');
ylim = get(gca,'Ylim');

elements = handles.elements;
x_d = linspace(xlim(1),xlim(2),elements);
y_d = linspace(ylim(1),ylim(2),elements);
[X,Y] = meshgrid(x_d,y_d);

gamma = [1  0 -1  0;
         0  1  0 -1];
x_start = handles.x;
y_start = handles.y;

axes(handles.axes1);
cla()
axis manual
axis([-2 4 -2 4])
view(2)
for i = 1:elements
    for j = 1:elements
        handles.x = X(i,j);
        handles.y = Y(i,j);
        guidata(hObject, handles);

        update_values_inverse(hObject, eventdata, handles);
        handles = guidata(hObject);
        getJacobian(hObject, eventdata, handles);
        handles = guidata(hObject);

        % iii) Manipulability ellipsoid
        for k = 1:length(gamma(1,:))
            line = handles.J*gamma(:,k);
%             figure(3)
            plot([X(i,j) X(i,j)+handles.mag*line(1)],[Y(i,j) Y(i,j)+handles.mag*line(2)],'k')
            hold on
            drawnow;
        end
            
    end
end

handles.x = x_start;
handles.y = y_start;
guidata(hObject, handles);

hold off
update_values_inverse(hObject, eventdata, handles);
message_out(hObject, eventdata, handles,'Done')
guidata(hObject, handles);


function mag_edit_Callback(hObject, eventdata, handles)
% hObject    handle to mag_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of mag_edit as text
%        str2double(get(hObject,'String')) returns contents of mag_edit as a double
handles.mag = str2double(get(hObject,'String'));
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function mag_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mag_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function elements_edit_Callback(hObject, eventdata, handles)
% hObject    handle to elements_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of elements_edit as text
%        str2double(get(hObject,'String')) returns contents of elements_edit as a double
handles.elements = str2double(get(hObject,'String'));
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function elements_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to elements_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in plot_work_space_button.
function plot_work_space_button_Callback(hObject, eventdata, handles)
% hObject    handle to plot_work_space_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.axes2);
plot(handles.d1max*cos(0:0.1:2.1*pi) + handles.rO1(1),handles.d1max*sin(0:0.1:2.1*pi) + handles.rO1(2),'b')
plot(handles.d2max*cos(0:0.1:2.1*pi) + handles.rO2(1),handles.d2max*sin(0:0.1:2.1*pi) + handles.rO2(2),'r')
plot(handles.d3max*cos(0:0.1:2.1*pi) + handles.rO3(1),handles.d3max*sin(0:0.1:2.1*pi) + handles.rO3(2),'g')
plot(handles.d4max*cos(0:0.1:2.1*pi) + handles.rO4(1),handles.d4max*sin(0:0.1:2.1*pi) + handles.rO4(2),'k')
guidata(hObject, handles);


% --- Executes on button press in reset_button.
function reset_button_Callback(hObject, eventdata, handles)
% hObject    handle to reset_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
GUI_4RP_OpeningFcn(hObject, eventdata, handles)%, varargin)



function control_gain_edit_Callback(hObject, eventdata, handles)
% hObject    handle to control_gain_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of control_gain_edit as text
%        str2double(get(hObject,'String')) returns contents of control_gain_edit as a double
handles.k_gain = str2double(get(hObject,'String'));
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function control_gain_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to control_gain_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function controls(hObject, eventdata, handles)
% Robot should be moved to the starting position if its doint open-loop
% If not, shift the manipulator from the desierd path to se how the 
% control scheme works.

% If this is is enabled the manipulator will start from a given position
%  switch handles.control_type
%      case 'Open_loop'     
%         handles.x = handles.x0*0.9;
%         handles.y = handles.y0;
%      case 'Closed_loop_joint'
%         handles.x = handles.x0*0.9;
%         handles.y = handles.y0;
%      case 'Closed_loop_task'
%         handles.x = handles.x0*0.9;
%         handles.y = handles.y0;
%  end

update_values_inverse(hObject, eventdata, handles);
handles = guidata(hObject); % Needed to get the most recent handles changed in update_values_forward()

theta0 = [handles.d1;      
          deg2rad(handles.theta1);  
          handles.d2;      
          deg2rad(handles.theta2);  
          handles.d3;      
          deg2rad(handles.theta3);  
          handles.d4;      
          deg2rad(handles.theta4)];

      
% Actuation joint1 is given by handles.i_dot
% Actuation joint2 is given by handles.j_dot
dt = 0.01;
% dt = 0.001;

tspan = [0:dt:3];
rO_V = [handles.rO1;handles.rO2;handles.rO3;handles.rO4];
message_out(hObject, eventdata, handles,'Stand by - Calculating path')

[T, theta] = ode45(@(t,theta0) theta_solve_control(t,theta0,handles.phi_dot,handles.major_axis,handles.minor_axis,handles.i_dot,handles.j_dot,handles.control_type,handles.k_gain,handles.a,handles.b,rO_V),tspan,theta0);

message_out(hObject, eventdata, handles,'Stand by - Animating path')
axes(handles.axes1);
cla()
view(2)
handles.d1 = theta(1,1);
handles.theta1 = rad2deg(theta(1,2));
handles.d2 = theta(1,3);
handles.theta2 = rad2deg(theta(1,4));
handles.d3 = theta(1,5);
handles.theta3 = rad2deg(theta(1,6));
handles.d4 = theta(1,7);
handles.theta4 = rad2deg(theta(1,8));
handles.x = handles.rO1(1) + theta(1,1)*cos(theta(1,2));
handles.y = handles.rO1(2) + theta(1,1)*sin(theta(1,2));
guidata(hObject, handles);
update_figure(hObject, eventdata, handles)
handles = guidata(hObject);

for i = 2:length(T)
    message_out(hObject, eventdata, handles,sprintf('Stand by - Animating path - t = %2.2f',T(i)))
    axes(handles.axes2);
    handles.d1 = theta(i,1);
    handles.theta1 = rad2deg(theta(i,2));
    handles.d2 = theta(i,3);
    handles.theta2 = rad2deg(theta(i,4));
    handles.d3 = theta(i,5);
    handles.theta3 = rad2deg(theta(i,6));
    handles.d4 = theta(i,7);
    handles.theta4 = rad2deg(theta(i,8));
    handles.x = handles.rO1(1) + theta(i,1)*cos(theta(i,2));
    handles.y = handles.rO1(2) + theta(i,1)*sin(theta(i,2));
    guidata(hObject, handles);
    update_figure(hObject, eventdata, handles)
    handles = guidata(hObject);
    axes(handles.axes2);
    plot([theta(i-1,1)*cos(theta(i-1,2)),theta(i,1)*cos(theta(i,2))],[theta(i-1,1)*sin(theta(i-1,2)) theta(i,1)*sin(theta(i,2))],'r')
    set(handles.end_affector,'XData',handles.x,'YData',handles.y)
    drawnow;
    
    axes(handles.axes1)
    axis([handles.a-handles.major_axis*1.3 handles.a+handles.major_axis*1.3 handles.b+-handles.minor_axis*1.3 handles.b+handles.minor_axis*1.3])
    hold on
    x_p = handles.a + handles.major_axis*cosd(handles.phi);
    y_p = handles.b +handles.minor_axis*sind(handles.phi);
    plot(x_p,y_p)
    drawnow;
    hold on
    axes(handles.axes1)
    plot([theta(i-1,1)*cos(theta(i-1,2)),theta(i,1)*cos(theta(i,2))],[theta(i-1,1)*sin(theta(i-1,2)) theta(i,1)*sin(theta(i,2))],'r','LineWidth',1.5)
    drawnow;

end

message_out(hObject, eventdata, handles,'Tracking done')
handles.x = handles.x0;
handles.y = handles.y0;
update_values_inverse(hObject, eventdata, handles);
guidata(hObject, handles);
        
function message_out(hObject, eventdata, handles,string)
set(handles.message_box,'String',string);
drawnow()
guidata(hObject, handles);

function plot_path(hObject, eventdata, handles)
x = handles.a + handles.major_axis*cosd(handles.phi);
y = handles.b + handles.minor_axis*sind(handles.phi);
set(handles.circle_plot,'XData',x,'YData',y)
handles.x0 = x(1);
handles.y0 = y(1);
guidata(hObject, handles)
