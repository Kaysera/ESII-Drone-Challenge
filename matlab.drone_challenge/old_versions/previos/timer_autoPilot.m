function t = timer_autoPilot(drone,h_ref)
t = timer;
t.StartFcn = @AutoPilotInit;
t.TimerFcn = {@AutoPilotCTRL,drone, h_ref};
t.StopFcn = @AutoPilotCleanup;
t.Period = 0.30;
t.StartDelay = 1;
%t.ExecutionMode = 'fixedSpacing';
t.ExecutionMode = 'fixedRate';
%t.ExecutionMode = 'singleShot';
t.BusyMode = 'error';
end 


function AutoPilotInit(~,~)
disp('AutoPilot started')
tic
end

function AutoPilotCTRL(this,~,drone,h_ref)
fprintf('AutoPilot CTRL--------------------------%.2f\n',toc);
tic;

getstate_time = tic;
[p,~,v] = getState(drone);  %position, orientation, velocity
fprintf('\t time getting state:%.2f',toc(getstate_time));

%disp(p)
%disp(v)
h = p(3);
h_dot = v.Linear(3);
fprintf('\t h:%.2f  \t h_dot:%.2f \n',h,h_dot);

impulse = -5*(h-h_ref) - 2*h_dot   + 9.8*1.840;
fprintf('\t impulse:%.2f\n',impulse);
if impulse > 0
    applyforce_time = tic;
%    applyForce(drone, 'fuselaje', this.Period, [0 0 impulse]);
    applyForce(drone, 'motor_NE', this.Period, [0 0 impulse/4]);
    applyForce(drone, 'motor_SW', this.Period, [0 0 impulse/4]);
    applyForce(drone, 'motor_NW', this.Period, [0 0 impulse/4]);
    applyForce(drone, 'motor_SE', this.Period, [0 0 impulse/4]);
    fprintf('\t time applying force:%.2f\n',toc(applyforce_time));
end

end

function AutoPilotCleanup(~,~)
disp('AutoPilot stopped')
end