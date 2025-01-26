%test02IssLVLHPointingSim  Test 02 for earth point sat with RW and PD Control
%-------------------------------------------------------------------------------
% Model
modelName = ('testIssLVLHPointingModel.slx');

%-------------------------------------------------------------------------------
% Simulation Parameters

% Intial Time [s]
t0 = 0;

% Final Time [s]
tFinal = 5400;

% Simulation time step [2]
dtSim = 0.1;

% Time series [s]
tVec = t0:dtSim:tFinal;

%-------------------------------------------------------------------------------
% Orbit parameters
mu = 3.986e14;
rho = 6600e3; %meters

satPosition_i = [400e3+rho 0 0]';
satVelocity = sqrt(mu/satPosition_i(1));

satPosition0 = satPosition_i;
satVelocity0 = [0 satVelocity 0]';


omega = sqrt(mu/satPosition0(1)^3);

%-------------------------------------------------------------------------------
% Sat Model Parameters

% Intial Satellite Attitude DCM <3,3>
rHat = satPosition0./norm(satPosition0);
vHat = satVelocity0./norm(satVelocity0);
crossVec = cross(-rHat, vHat);
yHat = crossVec./norm(crossVec);
satDynParams.satAttDcm0= [vHat'; yHat'; -rHat'];


% Inital Satellite Attitude Quaternion /vec s/
temp = dcm2quat(satDynParams.satAttDcm0);
 
satDynParams.satAttQuat0 = [temp(2:4)'; temp(1)]';

% Controller
adcsParams.kP = -2e-8*[1 1 1];
adcsParams.kD = -2e-8*[1 1 1];
adcsParams.innerAngDistGain = 1/dtSim;
adcsParams.outerAngDistGain  = 9e-10;
% Satellite Inertia Tensor
satDynParams.satInertiaTensor = [70942261    0 0; 0  59070966   0; 0   0 123909888];
%satDynParams.satInertiaTensor = [70942261    3512761   3543989; 3512761  59070966   -2768801; 3543989   -2768801  123909888];

% Initial Satellite angular rate
satDynParams.satAngRate0 = [0 -omega  0]';

% Reaction Wheels
hwParams.gimbalAngRates0 = [0 0 0 0 0 0 0 0]';
hwParams.gimbalAngles0 = [0 0 0 0 0 0 0 0]';
hwParams.rotorInertia =  6.8986;
hwParams.maxGimbalRates = .014;
hwParams.rotorAngRate = 691;
hwParams.gimbalErrorFlag = 1;
hwParams.gimbalBias = 0.001;
hwParams.gimbalNoisePower = 1e-3;

%Gyro parametrs
hwParams.gyroErrorFlag = 1;
hwParams.gyroBias = 1e-5;
hwParams.gyroNoisePower = [1e-11];

%Earth Horizon Parameters
hwParams.earthHorizonErrorFlag = 1;
sensorError = [ 0.03 0.03 0.03]';
eulerAng = sqrt(sensorError'*sensorError);
eulerAxis = 1/eulerAng.*[sensorError(1) sensorError(2) sensorError(3)]';
eulerAxSkew = [0 -eulerAxis(3) eulerAxis(2); eulerAxis(3) 0 -eulerAxis(1); -eulerAxis(2) eulerAxis(1) 0];

hwParams.earthHorizonErrorDcm = eye(3)-sin(eulerAng)*eulerAxSkew+(1-cos(eulerAng))*eulerAxSkew^2;

% Magnetometer Parameters
hwParams.magErrorFlag = 1;
hwParams.magError = [.1 .1 .1]';

%Distrubances
satDynParams.distTorqueFlag = 1;
%-------------------------------------------------------------------------------
% Test Inputs

% Initialize reference angular rate vector [rad/s] 

refSatAngRate_bi = zeros(length(tVec),3);
refSatAngRate_bi(:,2) = -omega;




%-------------------------------------------------------------------------------
% Run Test

results = sim(modelName);

%-------------------------------------------------------------------------------
% Analysis
close all
k = 0;
lw = 2;



satAttQuat_bi = results.satAttQuat_bi.data;
satAngRate_bi = results.satAngRate_bi.data;
cmgOutputTorque_b = results.cmgOutputTorque_b.data;
cmgAngRates = squeeze(results.gimbalAngles.data);
attitudeError = results.attitudeError.data;
attErrorRate = results.attErrorRate.data;

% Att Quat Plot

k = k+1;
figure(k)
subplot(4,1,1)
plot(tVec(:),satAttQuat_bi(:,1),LineWidth=lw)
grid on
title('Satellite Attitude Quaternion Intertial to Body Frame')
ylabel("e_1 Component")
subplot(4,1,2)
plot(tVec(:),satAttQuat_bi(:,2),LineWidth=lw)
grid on
ylabel("e_2 Component")
subplot(4,1,3)
plot(tVec(:),satAttQuat_bi(:,3),LineWidth=lw)
grid on
ylabel("e_3 Component")
subplot(4,1,4)
plot(tVec(:),satAttQuat_bi(:,4),LineWidth=lw)
grid on
ylabel("Scalar Component")
xlabel('Time [s]')

k=k+1;
figure(k)
hold on
plot(tVec,satAngRate_bi(:,1),LineWidth=lw)
plot(tVec,satAngRate_bi(:,2),LineWidth=lw)
plot(tVec,satAngRate_bi(:,3),LineWidth=lw)
plot(tVec,refSatAngRate_bi(:,2),'--g',LineWidth=lw)
grid on
hold off
legend('omega_x','omega_y','omega_z','Refernce y_b Angular Rate')
xlabel('Time [sec]')
ylabel('Angular Rate [rad/s]')
title('Satellite Angular Velocity body w.r.t inertial')

k=k+1;
figure(k)
hold on
plot(tVec,cmgOutputTorque_b(:,1),LineWidth=lw)
plot(tVec,cmgOutputTorque_b(:,2),LineWidth=lw)
plot(tVec,cmgOutputTorque_b(:,3),'.-',LineWidth=lw)
grid on
hold off
legend('m_x','m_y','m_z')
xlabel('Time [sec]')
ylabel('Torque [Nm]')
title('DGCMG Output Torque')

k=k+1;
figure(k)
hold on
plot(tVec,cmgAngRates(:,1),LineWidth=lw)
plot(tVec,cmgAngRates(:,2),LineWidth=lw)
plot(tVec,cmgAngRates(:,3),LineWidth=lw)
plot(tVec,cmgAngRates(:,4),LineWidth=lw)
plot(tVec,cmgAngRates(:,5),LineWidth=lw)
plot(tVec,cmgAngRates(:,6),LineWidth=lw)
plot(tVec,cmgAngRates(:,7),LineWidth=lw)
plot(tVec,cmgAngRates(:,8),LineWidth=lw)
grid on
hold off
xlabel('Time [sec]')
ylabel('Angle [rad]')
legend("alpha_1","beta_1","alpha_2","beta_2","alpha_3","beta_3","alpha_4","beta_4")
title('Inner and Outer Gimbal Angles')

k = k+1;
figure(k)
plot(tVec,attitudeError,LineWidth=lw)
legend('roll','pitch','yaw')
title("ISS Attitude Error Roll-Pitch-Yaw")
xlabel("Time [sec]")
ylabel("Error Magnitude")
grid on

k = k+1;
figure(k)
plot(tVec,attErrorRate,LineWidth=lw)
legend('roll','pitch','yaw')
title("ISS Attitude Error Rate Roll-Pitch-Yaw")
xlabel("Time [sec]")
ylabel("Error Rate Magnitude")
grid on



