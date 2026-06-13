function export_ref()
% Runs the hybrid loop of MAIN.m (no animation) and dumps the reference
% trajectory to ../mujoco/ref_matlab.csv to validate the MuJoCo port.
addpath gen
addpath fcns

p = get_params();
Nstep = 15;  p.Nstep = Nstep;
p.isMotorDynamics  = 1;     % with motor dynamics
p.isControlSaturate = 1;    % with 12V/30A saturation

q0  = [0; 0; pi/3; -pi/2];
dq0 = [0; 0; 0; 0];
ic  = [q0; dq0]';

tstart = 0;
tfinal = 2 * Nstep;
p.tTD  = zeros(2,0);

Xcur = ic;                 % current state (row)
% columns: t q1 q2 q3 q4 dq1 dq2 dq3 dq4 u1 u2 F1 F2 footx footy footz phase
LOG = zeros(0,17);

for istep = 1:Nstep
    %% flight phase (until touchdown: foot z = 0)
    opt = odeset('Events',@(t,X)event_touchDown(t,X,p),'MaxStep',0.005);
    [t,X] = ode45(@(t,X)dyn_aerial(t,X,p),[tstart, tfinal], Xcur(end,:), opt);
    p.tTD(:,end+1) = [t(end); 0];
    p.ptTD = fcn_p_toe(X(end,1:4), p.params);
    [~,u,F] = dyn_aerial(t,X,p);
    for ii = 1:length(t)
        q = X(ii,1:4)';  ft = fcn_p_toe(q, p.params);
        LOG(end+1,:) = [t(ii), X(ii,1:4), X(ii,5:8), u(ii,1), u(ii,2), ...
                        F(ii,1), F(ii,2), ft(1), ft(2), ft(3), 0];
    end
    tstart = t(end);

    %% impact map (hard inelastic contact)
    X_post = fcn_impactMap(X(end,:), p);
    Xcur = X_post';

    %% stance phase (until liftoff: GRFz < 1.5 N)
    opt = odeset('Events',@(t,X)event_liftOff(t,X,p),'MaxStep',0.005);
    [t,X] = ode45(@(t,X)dyn_stance(t,X,p),[tstart, tfinal], Xcur, opt);
    p.tLO = t(end);  p.tTD(2,end) = t(end);
    [~,u,F] = dyn_stance(t,X,p);
    for ii = 1:length(t)
        q = X(ii,1:4)';  ft = fcn_p_toe(q, p.params);
        LOG(end+1,:) = [t(ii), X(ii,1:4), X(ii,5:8), u(ii,1), u(ii,2), ...
                        F(ii,1), F(ii,2), ft(1), ft(2), ft(3), 1];
    end
    tstart = t(end);
    Xcur = X(end,:);
    fprintf('step %d/%d done\n', istep, Nstep);
end

hdr = {'t','q1','q2','q3','q4','dq1','dq2','dq3','dq4','u1','u2', ...
       'F1','F2','footx','footy','footz','phase'};
T = array2table(LOG, 'VariableNames', hdr);
writetable(T, '../mujoco/ref_matlab.csv');
fprintf('ref_matlab.csv written: %d rows\n', size(LOG,1));
end
