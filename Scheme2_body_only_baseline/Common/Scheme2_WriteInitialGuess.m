function Scheme2_WriteInitialGuess(x, y, theta, v, a, phy, w, time)
% Write initial guess for Scheme2 body-only NLP.
% No embodied-footprint variables are written.

global params

Nfe = numel(x);
params.nfe = Nfe;

x = x(:); y = y(:); theta = theta(:);
v = v(:); a = a(:); phy = phy(:); w = w(:);
dt = time(:);
if numel(dt) == Nfe
    dt = dt(1:end-1);
end

assert(numel(dt) == Nfe-1, 'time must have Nfe-1 intervals.');

[AX, AY, BX, BY, CX, CY, DX, DY] = bodyVertices(x(1:end-1), y(1:end-1), theta(1:end-1));

writeIg(Nfe, x, y, theta, v, a, phy, w, dt, AX, AY, BX, BY, CX, CY, DX, DY);
writePV(Nfe);

data = struct();
data.x = x; data.y = y; data.theta = theta;
data.v = v; data.a = a; data.phy = phy; data.w = w;
data.dt = dt; data.tf = sum(dt);
data.AX = AX; data.AY = AY; data.BX = BX; data.BY = BY;
data.CX = CX; data.CY = CY; data.DX = DX; data.DY = DY;
data.meta = struct('Nfe', Nfe, ...
    'task_id', params.task_id, ...
    'task_source', strtrim(getenv('EXPERIMENT_TASK_SOURCE')), ...
    'Nobs', params.environment.num_obs, ...
    'lw', params.vehicle.lw, ...
    'LF', params.vehicle.LF, ...
    'lr', params.vehicle.lr, ...
    'hlb', params.vehicle.hlb, ...
    'obs', params.environment.obs, ...
    'note', 'scheme2 body-only initial guess');
save('written_initial_guess_data_scheme2.mat', 'data');

% Scheme2 applies OBCA to the physical body at its equal-time nodes.
PrepareOBCAData('written_initial_guess_data_scheme2.mat', ...
    'ig_scheme2.INIVAL', 'OBCAData_scheme2.dat', 0.01);

fprintf('\n========== Scheme2 Initial Guess Written ==========\n');
fprintf('Nfe       : %d\n', Nfe);
fprintf('tf        : %.6f\n', sum(dt));
fprintf('dt range  : %.6e / %.6e\n', min(dt), max(dt));
fprintf('No embodied-footprint variables were written.\n');
fprintf('==================================================\n\n');
end

function writeIg(Nfe, x, y, theta, v, a, phy, w, dt, AX, AY, BX, BY, CX, CY, DX, DY)
if exist('ig_scheme2.INIVAL', 'file')
    delete('ig_scheme2.INIVAL');
end
fid = fopen('ig_scheme2.INIVAL', 'w');
if fid < 0
    error('Cannot create ig_scheme2.INIVAL.');
end
cleanup_obj = onCleanup(@() fclose(fid)); %#ok<NASGU>

for i = 1:Nfe
    fprintf(fid, 'let x[%d] := %.12f;\r\n', i, x(i));
    fprintf(fid, 'let y[%d] := %.12f;\r\n', i, y(i));
    fprintf(fid, 'let theta[%d] := %.12f;\r\n', i, theta(i));
    fprintf(fid, 'let v[%d] := %.12f;\r\n', i, v(i));
    fprintf(fid, 'let a[%d] := %.12f;\r\n', i, a(i));
    fprintf(fid, 'let phy[%d] := %.12f;\r\n', i, phy(i));
    fprintf(fid, 'let w[%d] := %.12f;\r\n', i, w(i));
end
for i = 1:Nfe-1
    fprintf(fid, 'let dt[%d] := %.12f;\r\n', i, dt(i));
    fprintf(fid, 'let AX[%d] := %.12f;\r\n', i, AX(i));
    fprintf(fid, 'let AY[%d] := %.12f;\r\n', i, AY(i));
    fprintf(fid, 'let BX[%d] := %.12f;\r\n', i, BX(i));
    fprintf(fid, 'let BY[%d] := %.12f;\r\n', i, BY(i));
    fprintf(fid, 'let CX[%d] := %.12f;\r\n', i, CX(i));
    fprintf(fid, 'let CY[%d] := %.12f;\r\n', i, CY(i));
    fprintf(fid, 'let DX[%d] := %.12f;\r\n', i, DX(i));
    fprintf(fid, 'let DY[%d] := %.12f;\r\n', i, DY(i));
end
fprintf(fid, 'let tf := %.12f;\r\n', sum(dt));
end

function writePV(Nfe)
global params
fid = fopen('PV_scheme2', 'w');
if fid < 0
    error('Cannot create PV_scheme2.');
end
cleanup_obj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '1  %.12f\r\n', params.task.x0);
fprintf(fid, '2  %.12f\r\n', params.task.y0);
fprintf(fid, '3  %.12f\r\n', params.task.theta0);
fprintf(fid, '4  %.12f\r\n', params.task.xf);
fprintf(fid, '5  %.12f\r\n', params.task.yf);
fprintf(fid, '6  %.12f\r\n', params.task.thetaf);
fprintf(fid, '7  %d\r\n', Nfe);
fprintf(fid, '8  %.12f\r\n', params.vehicle.v_max);
fprintf(fid, '9  %.12f\r\n', params.vehicle.phy_max);
fprintf(fid, '10 %.12f\r\n', params.vehicle.a_max);
fprintf(fid, '11 %.12f\r\n', params.vehicle.w_max);
fprintf(fid, '12 %.12f\r\n', params.vehicle.lw);
fprintf(fid, '13 %.12f\r\n', params.vehicle.LF);
fprintf(fid, '14 %.12f\r\n', params.vehicle.lr);
fprintf(fid, '15 %.12f\r\n', params.vehicle.hlb);
fprintf(fid, '16 %d\r\n', params.environment.num_obs);
fprintf(fid, '17 %.12f\r\n', params.ef.max_dt);
fprintf(fid, '18 %.12f\r\n', params.ef.min_dt);
end

function [AX, AY, BX, BY, CX, CY, DX, DY] = bodyVertices(x, y, theta)
global params
LF = params.vehicle.LF;
lr = params.vehicle.lr;
hlb = params.vehicle.hlb;
c = cos(theta); s = sin(theta);
AX = x + LF .* c - hlb .* s;
AY = y + LF .* s + hlb .* c;
BX = x + LF .* c + hlb .* s;
BY = y + LF .* s - hlb .* c;
CX = x - lr .* c + hlb .* s;
CY = y - lr .* s - hlb .* c;
DX = x - lr .* c - hlb .* s;
DY = y - lr .* s + hlb .* c;
end
