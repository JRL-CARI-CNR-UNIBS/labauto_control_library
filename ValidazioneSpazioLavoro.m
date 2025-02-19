%% Validazione: modello in diversi punti di lavoro.
% Carico i risultati del test nei vari working point (wp)

clc;clear all;close all;

giunto="secondo";
if giunto=="primo"
    load modello_primo_giunto.mat
    tests=dir('tests/wp_validation_chirp_experiment_joint1_*.mat');
else
    load modello_secondo_giunto.mat
    tests=dir('tests/wp_validation_chirp_experiment_joint2_*.mat');
end    

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

    figure(1)
    subplot(3,1,1)
    plot(time,joint_position(:,ngiunto))
    xlabel('Time')
    ylabel('Position')
    hold on
    subplot(3,1,2)
    plot(time,output)
    xlabel('Time')
    ylabel('Velocity')
    hold on
    subplot(3,1,3)
    plot(time,control_action)
    xlabel('Time')
    ylabel('Torque')
    hold on
                
    figure(2)
    bode_opts = bodeoptions('cstprefs');
    bode_opts.PhaseWrapping = 'on';
    h=bodeplot(freq_resp_valid,'k', bode_opts);
    grid on
    hold on
end
figure(2)
plot(w0*[1 1],ylim,'--r')
plot(w1*[1 1],ylim,'--r')

bode_opts = bodeoptions('cstprefs');
bode_opts.PhaseWrapping = 'on';

bode(modello_continuo,bode_opts)

%% 
%