clear all, close all

%% electrical parameters
mot.p = 5; % Motor pole pairs
mot.R = 0.342; %[Ohm] stator resistance
mot.lammg = 0.0082; %[Vs] 
mot.Ld = 0.32e-3; %[H]
mot.Lq = 0.32e-3; %[H]

%% mechanical parameters 
mot.B = 2.79e-5; % (N*m*s/rad) Viscous friction
mot.J = 1.77e-5;

% characteristics parameters
mot.IN = 4.53; % (A) motor rated currents
mot.UN = 20; % (V) motor rated voltage
mot.WN = 2.8e3; % (rpm) motor rated speed
mot.tauN = 0.22; % (Nm) motor rated torque
mot.kTau=1.5*mot.p*mot.lammg; %[Vs] torque constant

%% lookup tables
idVec = -5:1:5; %[A]
iqVec = -10:1:10; %[A]

lamdVec = mot.Ld*idVec + mot.lammg;
lamqVec = mot.Lq*iqVec;

[lamdMap,lamqMap] = meshgrid(lamdVec,lamqVec);
[idMap,iqMap] = meshgrid(idVec,iqVec);

%% INVERTER and NON IDEALITIES
inv.Ubus   = 24;        %[V]  bus voltage
inv.Fs     = 8e3; % (Hz) Sampling frequency
inv.Ts     = 1/inv.Fs; % (s) Time period of the sampling
inv_Ts=inv.Ts;
inv.tauD=1.5*inv.Ts;

inv.Fpwm = 8e3; % (Hz) Switching frequency
inv.Tpwm = 1/inv.Fpwm; % (s) Time period of the switching

inv.nid.Vsw = 0.08; % (V) Switch voltage drop
inv.nid.Vf = 1.5; % (V) Freewheeling diode voltage drop
inv.nid.Vcm = (inv.nid.Vf+inv.nid.Vsw)/2;
inv.nid.Vdm = (inv.nid.Vf-inv.nid.Vsw)/2;

inv.nid.DT = 1.5e-6; % (s) dead time
inv.nid.toff = 0.34e-6; % (s) turn off time of the switch
inv.nid.ton = 0.2e-6; % (s) turn on time of the switch
inv.nid.ttot = inv.nid.DT + inv.nid.ton - inv.nid.toff;

%% PI cur
PI.cur.wgc = 200*(2*pi); % (rad/s) Control bandwidth
PI.cur.phim = 80*(pi/180); % (rad) Phase margin
PI.cur.Ts = inv.Tpwm;
inv.tauD = 1.5*inv.Tpwm;

%continuous case gains
%[PI.cur.kpd, PI.cur.kid] = getPiCur(PI.cur.wgc, PI.cur.phim, mot.R, mot.Ld, inv.tauD);
%[PI.cur.kpq, PI.cur.kiq] = getPiCur(PI.cur.wgc, PI.cur.phim, mot.R, mot.Lq, inv.tauD);

%discrete case gains
[PI.cur.kpd, PI.cur.kid] = getPiCur(PI.cur.wgc, PI.cur.phim, mot.R, mot.Ld, inv.tauD, PI.cur.Ts);
[PI.cur.kpq, PI.cur.kiq] = getPiCur(PI.cur.wgc, PI.cur.phim, mot.R, mot.Lq, inv.tauD, PI.cur.Ts);

PI.cur.Ld = mot.Ld;
PI.cur.Lq = mot.Lq;
PI.cur.lammg = mot.lammg;

%% PI VEL
PI.vel.fs = 12.5e3;
PI.vel.Ts = 1/PI.vel.fs;
PI.vel.wgc = 20*(2*pi);
PI.vel.phim = 80*pi/180;
[PI.vel.kp, PI.vel.ki] = getPiVel(PI.vel.wgc,PI.vel.phim,mot.B,mot.J,1/PI.cur.wgc,PI.vel.Ts);
PI.vel.kt = PI.vel.ki/PI.vel.kp;
%% MEASURES
%Current measurement acquisition
meas.cur.adc.FS = 8.8; % (A) Full scale (+- 4.4A)
meas.cur.adc.Nb = 12; % ADC number of bit
meas.cur.adc.q = meas.cur.adc.FS/2^meas.cur.adc.Nb; % (A)

%Encoder measurement acquisition
meas.enc.Np = 250; % Number of pulses
meas.enc.q = 2*pi/meas.enc.Np;

meas.enc.nTabs = 40; %buffer size
%meas.enc.Ts = PI.vel.Ts; %sampling time
meas.enc.Ts = inv.Ts;

z = tf('z',inv.Ts); % Z-transfom variable

%% sensorless PLL
SL.Ts = inv.Tpwm;
SL.sat_min = 0.001;

SL.PI.wgc = 80*(2*pi);
SL.PI.phim = 80*(pi/180);
SL.tau = 1/(2*pi/SL.Ts);
SL.alpha = SL.tau/(SL.tau + SL.Ts);

s = tf('s');
Process = 1/s * 1/(1+ s*inv.tauD) * 1/(1 + s*SL.tau);
[SL.PI.kp, SL.PI.ki] = getPI(Process, SL.PI.phim, SL.PI.wgc, SL.Ts);
[SL.PI.kp, SL.PI.ki]

