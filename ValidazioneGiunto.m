%% Validazione in presenza di limitazioni di velocità/posizione massima.
% In questo script vedremo la validazione con sweep in frequenza (chirp)
% 
% Carico i risultati del test

clc;clear all;close all;
model_name='scara0';
giunto="primo";
if giunto=="primo"
    load([model_name,'/modello_primo_giunto'])
    tests=dir([model_name,'/tests/validation_chirp_experiment_joint1*.mat']);
else
    load([model_name,'/modello_secondo_giunto.mat'])
    tests=dir([model_name,'/tests/validation_chirp_experiment_joint2*.mat']);
end    

bode_opts = bodeoptions('cstprefs');
bode_opts.PhaseWrapping = 'on';

for itest=1:length(tests)
    load([tests(itest).folder,filesep,tests(itest).name])
    fprintf('Giunto=%d\n',joint_number+1);
    fprintf('Chirp con Ampiezza %f da %f a %f\n',A,f0,f1)
    punto_di_lavoro=mean(joint_position);
    fprintf('Punto di lavoro [%f %f]\n',punto_di_lavoro(1),punto_di_lavoro(2))


    ngiunto=joint_number+1;
    Tc=time(2)-time(1);


    w0=2*pi*f0; %rad/s
    w1=2*pi*f1; %rad/s     Fs=1/Ts, Ws=2*pi/Ts, max w1 = 0.5*Ws=pi/Ts
    control_action=joint_torque(:,ngiunto);
    output=joint_velocity(:,ngiunto);
    
    validation=iddata(output,control_action,Tc);
    freq_resp_valid = spafdr(validation);
    
    figure
    h=bodeplot(freq_resp_valid,'k', bode_opts);
    grid on
    hold on
    showConfidence(h,3)
    plot(w0*[1 1],ylim,'--r')
    plot(w1*[1 1],ylim,'--r')
    bode(modello_continuo,bode_opts)
    drawnow
    xlim(sort([w0 w1]))
end

%%