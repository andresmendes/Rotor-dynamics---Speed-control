%% Rotor dynamics - Speed control
% Simulation and animation of a controlled rotor with speed tracking and
% disturbance rejection.
%
%%

clear ; close all ; clc

%% Circle
th_c    = 0:0.01:2*pi+0.1;      % angle for sweep               [rad]
r       = 1;                    % Radius                        [m]
x_c     = r*cos(th_c);          % Position x                    [m]
y_c     = r*sin(th_c);          % Position y                    [m]

%% Simulation
tf      = 60;                   % Final time                    [s]
fR      = 30;                   % Frame rate                    [fps]
dt      = 1/fR;                 % Time resolution               [s]
time    = linspace(0,tf,tf*fR); % Time                          [s]

% Initial conditions [position speed]
th_0    = 0;                    % Initial angular position      [rad]
w_0     = 0;                    % Initial angular speed         [rad/s]
th_ref0 = 0;                    % Initial angular pos. ref.     [rad/s]
z0      = [th_0 w_0 th_ref0];

options = odeset('RelTol',1e-6);
[T, Z]  = ode45(@rotor_dynamics, time, z0,options);

%% Retrieving states
th      = Z(:,1);               % angular position              [rad]
w       = Z(:,2);               % angular speed                 [rad/s]
% Preallocating
torque      = zeros(1,length(T));
disturbance = zeros(1,length(T));
w_ref       = zeros(1,length(T));
for i=1:length(T)
  [dz, torque(i), disturbance(i), w_ref(i)] = rotor_dynamics(T(i),Z(i,:));
end

% Rotation indicator
x = r*cos(th);                  % Coordinate x                  [m]
y = r*sin(th);                  % Coordinate y                  [m]

%% Animation

figure
set(gcf,'Position',[270   140   640     360  ])

% Create and open video writer object
v = VideoWriter('rotor_dynamics_speed.avi');
v.Quality = 100;
open(v);

for i=1:length(time)
    subplot(2,2,1)
      cla
      hold on ; grid on ; axis equal ; box on
      set(gca,'xlim',[-1.2*r 1.2*r],'ylim',[-1.2*r 1.2*r])
      set(gca,'XTick',[], 'YTick', [])
      plot(x_c,y_c,'k','linewidth',3)               % Circle
      plot([-x(i) x(i)],[-y(i) y(i)],'r--')         % Rotation indicator
      plot(0,0,'k*')                                % Origin
      title('Rotor')
    subplot(2,2,2)
      cla
      hold on ; grid on ; box on
      set(gca,'xlim',[T(1) T(end)],'ylim',[min(torque)-0.1*max(torque) max(torque)+0.1*max(torque)])
      plot(T,torque,'b','LineWidth',1.5)
      plot([T(i) T(i)],[min(torque)-0.1*max(torque) max(torque)+0.1*max(torque)],'k--')
      xlabel('Time [s]')
      ylabel('Torque [N.m]')
    subplot(2,2,3)
      cla
      hold on ; grid on ; box on
      set(gca,'xlim',[T(1) T(end)],'ylim',[min(disturbance)-0.1*max(disturbance) max(disturbance)+0.1*max(disturbance)])
      plot(T,disturbance,'b','LineWidth',1.5)
      plot([T(i) T(i)],[min(disturbance)-0.1*max(disturbance) max(disturbance)+0.1*max(disturbance)],'k--')
      xlabel('Time [s]')
      ylabel('Disturbance [N.m]')
    subplot(2,2,4)
      cla
      hold on ; grid on ; box on
      set(gca,'xlim',[T(1) T(end)],'ylim',[min(w)-0.1*max(w) max(w)+0.1*max(w)])
      plot(T,w,'b','LineWidth',1.5)
      plot(T,w_ref,'k')
      plot([T(i) T(i)],[min(w)-0.1*max(w) max(w)+0.1*max(w)],'k--')
      xlabel('Time [s]')
      ylabel('Angular speed [rad/s]')
      title('Black=Reference, Blue=Actual')
  
    frame = getframe(gcf);
    writeVideo(v,frame);
end

close(v);

%% Auxiliary functions

function [dz, torque, disturbance, w_ref] = rotor_dynamics(t,z)

    % Parameters
    I   = 2;                    % Moment of inertia             [kg.m2]
    c   = 1;                    % Viscous friction constant     [N.m.s/rad]
    
    th      = z(1);             % Angular position              [rad]
    w       = z(2);             % Angular speed                 [rad/s]
    th_ref  = z(3);             % Angular speed                 [rad/s]

    % Disturbance [N.m]
    if t<15
        disturbance = 0;
    elseif t<30
        disturbance = 10;

    else
        disturbance = 0;
    end

    % Controller
    % Reference angular speed [rad/s]
    if t<45
        w_ref = 8;                  
    else
        w_ref = 4;
    end
    Kp = 1.5;                   % Proportional gain
    Ki = 0.8;                   % Integral gain
    torque = Kp*(w_ref - w) + Ki*(th_ref - th); % [N.m]

    % Rotor dynamics
    dz(1,1) = w;
    dz(2,1) = ((torque - disturbance) - c*w)/I;
    dz(3,1) = w_ref;
  
end
