function build_pendubot_math_plant()
% 最稳版本：使用 Interpreted MATLAB Function，不指定维度
% 自动创建 pendubot_math_plant.slx（数学 Pendubot 模型）

modelName = 'pendubot_math_plant';

% cleanup
try, close_system(modelName,0); end
new_system(modelName);
open_system(modelName);

%% layout
x0 = 30; y0 = 80; dx = 140;

%% tau Inport
add_block('simulink/Sources/In1', [modelName '/tau'], ...
    'Position', [x0 y0 x0+40 y0+20]);

%% Interpreted MATLAB Function (no dimension settings!)
mf_x = x0 + dx;
mf_y = y0 - 20;

mf = add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', ...
    [modelName '/pendubot_dyn_block'], ...
    'Position', [mf_x mf_y mf_x+260 mf_y+80]);

set_param(mf, 'MATLABFcn', 'pendubot_dyn');   % 就这一句

%% Integrator for dq
int_dq_x = mf_x + dx + 50;
int_dq_y = y0 - 40;

add_block('simulink/Continuous/Integrator', ...
    [modelName '/Int_dq'], ...
    'Position', [int_dq_x int_dq_y int_dq_x+40 int_dq_y+60], ...
    'InitialCondition', '[0;0]');

%% Integrator for q
int_q_x = int_dq_x + dx;
int_q_y = y0 - 40;

add_block('simulink/Continuous/Integrator', ...
    [modelName '/Int_q'], ...
    'Position', [int_q_x int_q_y int_q_x+40 int_q_y+60], ...
    'InitialCondition', '[0;0]');

%% Demux q
add_block('simulink/Signal Routing/Demux', ...
    [modelName '/Demux_q'], ...
    'Outputs', '2', ...
    'Position', [int_q_x+70 int_q_y int_q_x+100 int_q_y+60]);

%% Demux dq
add_block('simulink/Signal Routing/Demux', ...
    [modelName '/Demux_dq'], ...
    'Outputs', '2', ...
    'Position', [int_q_x+70 int_q_y+80 int_q_x+100 int_q_y+140]);

%% Outports
add_block('simulink/Sinks/Out1', [modelName '/q1'], ...
    'Position', [int_q_x+140 int_q_y int_q_x+180 int_q_y+20]);
add_block('simulink/Sinks/Out1', [modelName '/q2'], ...
    'Position', [int_q_x+140 int_q_y+30 int_q_x+180 int_q_y+50]);
add_block('simulink/Sinks/Out1', [modelName '/dq1'], ...
    'Position', [int_q_x+140 int_q_y+80 int_q_x+180 int_q_y+100]);
add_block('simulink/Sinks/Out1', [modelName '/dq2'], ...
    'Position', [int_q_x+140 int_q_y+110 int_q_x+180 int_q_y+130]);

%% Connections
add_line(modelName, 'tau/1', 'pendubot_dyn_block/3');
add_line(modelName, 'Int_q/1',  'pendubot_dyn_block/1');
add_line(modelName, 'Int_dq/1', 'pendubot_dyn_block/2');

% ddq → dq integrator
add_line(modelName, 'pendubot_dyn_block/1', 'Int_dq/1');

% dq → q integrator
add_line(modelName, 'Int_dq/1', 'Int_q/1');

% q Demux
add_line(modelName, 'Int_q/1', 'Demux_q/1');
add_line(modelName, 'Demux_q/1', 'q1/1');
add_line(modelName, 'Demux_q/2', 'q2/1');

% dq Demux
add_line(modelName, 'Int_dq/1', 'Demux_dq/1');
add_line(modelName, 'Demux_dq/1', 'dq1/1');
add_line(modelName, 'Demux_dq/2', 'dq2/1');

%% save
save_system(modelName);
fprintf('Model "%s.slx" created.\n', modelName);

end
