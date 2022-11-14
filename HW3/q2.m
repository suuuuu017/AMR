clear
close all
clc

%% Parameters

% Workspace Size
xlim([0 100])
ylim([0 100])

lmx = [25, 25, 70, 70, 10, 80];
lmy = [25, 70, 25, 70, 40, 60];

% % Initialize a vector of positions for the robot
% x=[]; 
% y=[];
% theta = [];
x = 0;
y = 0;
theta = 0;

% Robot Initial Pose
if rand > 0.5 
    x = 50 + 10 * rand;
else
    x = 50 - 10 * rand;
end
if rand > 0.5 
    y = 50 + 10 * rand;
else
    y = 50 - 10 * rand;
end

% Initial Orientation 
theta = 2*pi*rand;

% Distribution Constant
% TODO: why
sigma = 8;

% Initial Particles 
particleN = 1000;

partX = rand(1000, 1) * 100;
partY = rand(1000, 1) * 100;
partTheta = rand(1000, 1) * 2 * pi;
particles = cat(2, partX, partY, partTheta);

weight = ones(1000, 1);

% Initial Guess
guess = mean(particles);

error = [];
errorX = [];
errorY = [];

angerror = [];

guessCol = [];

for step = 1:10
    % Robot moves forward
%     tmptheta = theta + 0.2;
    newTheta = theta + 0.2 + normrnd(0,0.5);
    if newTheta > 2*pi
        newTheta = newTheta - 2*pi;
    end
    vel = 1;
    vel = vel + normrnd(0,0.5);
    x = x + cos(newTheta) * vel;
    y = y + sin(newTheta) * vel;

    % Particles move forward
    % TODO: Do all particles have the same noise


    % Compute weight
    for i = 1:particleN
        prob = 1;
        particles(i,3) = particles(i,3) + 0.2 + normrnd(0,0.5);
        if particles(i,3) > 2*pi
            particles(i,3) = particles(i,3) - 2*pi;
        end
        velP = 1 + normrnd(0,0.5);
        particles(i,1) = particles(i,1) + ...
            cos(particles(i,3)) * (velP);
        particles(i,2) = particles(i,2) + ...
            sin(particles(i,3)) * (velP);
        for lm = 1:6
            dist = sqrt((particles(i,1) - lmx(lm))^2 ...
                + (particles(i,2) - lmy(lm))^2);
            meas = sqrt((x - lmx(lm))^2 ...
                + (y - lmy(lm))^2) + normrnd(0,8);
            gaussian = 1 / sqrt(2 * pi * sigma ^ 2) * ...
                exp(-1 / 2 * (dist - meas) ^ 2 / sigma ^ 2);
            prob = prob * gaussian;
        end
        weight(i) = prob;
    end

    % Normalize Weights
    weight = weight / sum(weight);

    % Resampling
    % TODO: mathematical proof of resampling
    newParticles = zeros(particleN,3);
    index = randi(particleN);
    beta = 0;
    for i = 1:particleN
        beta = beta + rand * 2 * max(weight);
        while weight(index) < beta
            beta = beta - weight(index);
            index = mod((index + 1), particleN);
            if index == 0
                index = 1;
            end
        end
        newParticles(i, :) = particles(index, :);
    end
    particles = newParticles;
    guess = mean(particles);
    guessCol = [guessCol, guess];
    
    plot(particles(:, 1), particles(:, 2), '.');
%     hold on;
%     robot = TriangularRobot(guess(1), guess(2), guess(3));
%     plot(robot(:,1),robot(:,2),'-');
    xlim([0 100]);
    ylim([0 100]);
%     hold off;
%     pause(0.01)
%     figure;
    
    errorX = [errorX, abs(guess(1) - x)];
    errorY = [errorY, abs(guess(2) - y)];
    error = [error, sqrt((guess(1) - x) ^ 2 + (guess(2) - y) ^ 2)]
    angerror = [angerror, abs(guess(3) - theta)];

end

figure;
plot(particles(:, 1), particles(:, 2), '.');
figure;
plot(error);
figure;
plot(errorX);
figure;
plot(errorY);
figure;
plot(angerror);


