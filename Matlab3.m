
%% 1. Fuzzy Inference System Creation
fis = mamfis('Name','ObstacleAvoidance');

% Configure FIS operations
fis.DefuzzificationMethod = 'centroid';
fis.AndMethod = 'min';
fis.OrMethod = 'max'; 
fis.ImplicationMethod = 'min';
fis.AggregationMethod = 'max';

%% 2. Input/Output Variables with Membership Functions
% Distance input (0-200 cm)
fis = addInput(fis, [0 200], 'Name', 'Distance');
fis = addMF(fis, 'Distance', 'trapmf', [0 0 30 50], 'Name', 'VeryClose');
fis = addMF(fis, 'Distance', 'trimf', [40 70 100], 'Name', 'Close');
fis = addMF(fis, 'Distance', 'trimf', [80 120 160], 'Name', 'Medium');
fis = addMF(fis, 'Distance', 'trapmf', [150 170 200 200], 'Name', 'Far');

% Angle input (-180 to 180 degrees)
fis = addInput(fis, [-180 180], 'Name', 'Angle');
fis = addMF(fis, 'Angle', 'trapmf', [-180 -180 -45 -20], 'Name', 'Left');
fis = addMF(fis, 'Angle', 'trimf', [-30 0 30], 'Name', 'Front');
fis = addMF(fis, 'Angle', 'trapmf', [20 45 180 180], 'Name', 'Right');

% Steering output (-45 to 45 degrees)
fis = addOutput(fis, [-45 45], 'Name', 'Steering');
fis = addMF(fis, 'Steering', 'trimf', [-45 -45 -22.5], 'Name', 'HardLeft');
fis = addMF(fis, 'Steering', 'trimf', [-45 -22.5 0], 'Name', 'Left');
fis = addMF(fis, 'Steering', 'trimf', [-22.5 0 22.5], 'Name', 'Straight');
fis = addMF(fis, 'Steering', 'trimf', [0 22.5 45], 'Name', 'Right');
fis = addMF(fis, 'Steering', 'trimf', [22.5 45 45], 'Name', 'HardRight');

% Speed output (0-100%)
fis = addOutput(fis, [0 100], 'Name', 'Speed');
fis = addMF(fis, 'Speed', 'trapmf', [0 0 10 20], 'Name', 'Stop');
fis = addMF(fis, 'Speed', 'trimf', [15 30 45], 'Name', 'Slow');
fis = addMF(fis, 'Speed', 'trimf', [40 60 80], 'Name', 'Medium');
fis = addMF(fis, 'Speed', 'trapmf', [70 90 100 100], 'Name', 'Fast');

%% 3. Rule Base
rules = [
    "Distance==VeryClose & Angle==Left => Steering=HardRight, Speed=Stop (1)"
    "Distance==VeryClose & Angle==Front => Steering=HardLeft, Speed=Stop (1)"
    "Distance==VeryClose & Angle==Right => Steering=HardLeft, Speed=Stop (1)"
    "Distance==Close & Angle==Left => Steering=Right, Speed=Slow (1)"
    "Distance==Close & Angle==Front => Steering=Left, Speed=Slow (1)"
    "Distance==Close & Angle==Right => Steering=Left, Speed=Slow (1)"
    "Distance==Medium & Angle==Left => Steering=Straight, Speed=Medium (1)"
    "Distance==Medium & Angle==Front => Steering=Straight, Speed=Medium (1)"
    "Distance==Medium & Angle==Right => Steering=Straight, Speed=Medium (1)"
    "Distance==Far => Steering=Straight, Speed=Fast (1)"
    "Distance==VeryClose => Speed=Stop (1)" % Safety override
];

fis = addRule(fis, rules);

%% 4. Operational Scenario Analysis
% Define a scenario where robot approaches an obstacle head-on
scenario_distance = linspace(200, 0, 50); % From 200cm to 0cm
scenario_angle = zeros(size(scenario_distance)); % Straight ahead
outputs = zeros(length(scenario_distance), 2);

% Storage for rule activations
rule_strengths = zeros(length(scenario_distance), length(fis.Rules));

for i = 1:length(scenario_distance)
    % Evaluate FIS
    [outputs(i,:), ~, ~, rule_strengths(i,:)] = evalfis(fis, [scenario_distance(i), scenario_angle(i)]);
end

%% 5. Visualization of Results
% Plot membership functions
figure;
subplot(2,1,1); plotmf(fis,'input',1); title('Distance MF');
subplot(2,1,2); plotmf(fis,'input',2); title('Angle MF');

figure;
subplot(2,1,1); plotmf(fis,'output',1); title('Steering MF');
subplot(2,1,2); plotmf(fis,'output',2); title('Speed MF');

% Plot control surfaces
figure;
gensurf(fis,[1 2],1); 
title('Steering Control Surface');
xlabel('Distance (cm)'); ylabel('Angle (deg)'); zlabel('Steering (deg)');

figure;
gensurf(fis,[1 2],2);
title('Speed Control Surface');
xlabel('Distance (cm)'); ylabel('Angle (deg)'); zlabel('Speed (%)');

% Plot scenario results
figure;
subplot(3,1,1);
plot(scenario_distance, outputs(:,1));
xlabel('Distance to Obstacle (cm)');
ylabel('Steering (deg)');
title('Controller Output - Steering');
grid on;

subplot(3,1,2);
plot(scenario_distance, outputs(:,2));
xlabel('Distance to Obstacle (cm)');
ylabel('Speed (%)');
title('Controller Output - Speed');
grid on;

% Plot rule activations
subplot(3,1,3);
plot(scenario_distance, rule_strengths);
xlabel('Distance to Obstacle (cm)');
ylabel('Rule Activation Strength');
title('Rule Activations During Approach');
legend({fis.Rules.Description}, 'Interpreter', 'none', 'Location', 'eastoutside');
grid on;

%% 6. Detailed Rule Analysis at Critical Points
critical_distances = [150, 100, 60, 30, 10]; % cm
critical_angles = [0]; % Straight ahead

for dist = critical_distances
    [output, ~, ~, rule_str] = evalfis(fis, [dist, 0]);
    
    figure;
    
    % Plot input membership grades
    subplot(3,1,1);
    plotmf(fis, 'input', 1);
    hold on;
    line([dist dist], [0 1], 'Color', 'r', 'LineWidth', 2);
    title(sprintf('Distance MF Activation at %.1fcm', dist));
    
    % Plot rule strengths
    subplot(3,1,2);
    bar(rule_str);
    set(gca, 'XTick', 1:length(fis.Rules));
    set(gca, 'XTickLabel', {fis.Rules.Description}, 'XTickLabelRotation', 90);
    ylabel('Activation Strength');
    title('Rule Activations');
    
    % Plot output fuzzy sets
    subplot(3,1,3);
    plotmf(fis, 'output', 1);
    hold on;
    line([output(1) output(1)], [0 1], 'Color', 'r', 'LineWidth', 2);
    title(sprintf('Steering Output (Crisp=%.1f°)', output(1)));
    
    set(gcf, 'Position', [100 100 800 800]);
end