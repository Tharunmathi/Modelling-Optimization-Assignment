%% Mamdani Fuzzy Logic Controller for Robot Obstacle Avoidance
% This system takes distance and angle to obstacles as inputs and outputs
% steering and speed adjustments using Mamdani fuzzy inference.


%% Step 1: Create the Fuzzy Inference System (FIS)
fis = mamfis('Name', 'ObstacleAvoidance');

% Configure FIS operations
fis.DefuzzificationMethod = 'centroid';  % For crisp output
fis.AndMethod = 'min';                  % AND operation (rule antecedents)
fis.OrMethod = 'max';                   % OR operation (rule antecedents)
fis.ImplicationMethod = 'min';          % Rule strength application
fis.AggregationMethod = 'max';          % Combine rule outputs

%% Step 2: Add Input Variables and Membership Functions

% Input 1: Distance to obstacle (0-100 cm)
fis = addInput(fis, [0 100], 'Name', 'Distance');
fis = addMF(fis, 'Distance', 'trapmf', [0 0 20 30], 'Name', 'VeryClose');
fis = addMF(fis, 'Distance', 'trimf', [20 40 60], 'Name', 'Close');
fis = addMF(fis, 'Distance', 'trimf', [50 70 90], 'Name', 'Medium');
fis = addMF(fis, 'Distance', 'trapmf', [80 90 100 100], 'Name', 'Far');

% Input 2: Angle to obstacle (-180 to 180 degrees)
fis = addInput(fis, [-180 180], 'Name', 'Angle');
fis = addMF(fis, 'Angle', 'trapmf', [-180 -180 -60 -30], 'Name', 'Left');
fis = addMF(fis, 'Angle', 'trimf', [-45 0 45], 'Name', 'Front');
fis = addMF(fis, 'Angle', 'trapmf', [30 60 180 180], 'Name', 'Right');

%% Step 3: Add Output Variables and Membership Functions

% Output 1: Steering adjustment (-45 to 45 degrees)
fis = addOutput(fis, [-45 45], 'Name', 'SteeringAdjustment');
fis = addMF(fis, 'SteeringAdjustment', 'trimf', [-45 -30 -15], 'Name', 'HardLeft');
fis = addMF(fis, 'SteeringAdjustment', 'trimf', [-30 -15 0], 'Name', 'Left');
fis = addMF(fis, 'SteeringAdjustment', 'trimf', [-15 0 15], 'Name', 'Straight');
fis = addMF(fis, 'SteeringAdjustment', 'trimf', [0 15 30], 'Name', 'Right');
fis = addMF(fis, 'SteeringAdjustment', 'trimf', [15 30 45], 'Name', 'HardRight');

% Output 2: Speed adjustment (0-100% of max speed)
fis = addOutput(fis, [0 100], 'Name', 'SpeedAdjustment');
fis = addMF(fis, 'SpeedAdjustment', 'trapmf', [0 0 20 30], 'Name', 'Stop');
fis = addMF(fis, 'SpeedAdjustment', 'trimf', [20 40 60], 'Name', 'Slow');
fis = addMF(fis, 'SpeedAdjustment', 'trimf', [50 70 90], 'Name', 'Medium');
fis = addMF(fis, 'SpeedAdjustment', 'trapmf', [80 90 100 100], 'Name', 'Fast');

%% Step 4: Define the Rule Base
rules = [
    % VeryClose distance rules (aggressive avoidance)
    "Distance==VeryClose & Angle==Left => SteeringAdjustment=HardRight, SpeedAdjustment=Stop (1)"
    "Distance==VeryClose & Angle==Front => SteeringAdjustment=HardLeft, SpeedAdjustment=Stop (1)"
    "Distance==VeryClose & Angle==Right => SteeringAdjustment=HardLeft, SpeedAdjustment=Stop (1)"
    
    % Close distance rules (gentle avoidance)
    "Distance==Close & Angle==Left => SteeringAdjustment=Right, SpeedAdjustment=Slow (1)"
    "Distance==Close & Angle==Front => SteeringAdjustment=Left, SpeedAdjustment=Slow (1)"
    "Distance==Close & Angle==Right => SteeringAdjustment=Left, SpeedAdjustment=Slow (1)"
    
    % Medium distance rules (maintain course)
    "Distance==Medium & Angle==Left => SteeringAdjustment=Straight, SpeedAdjustment=Medium (1)"
    "Distance==Medium & Angle==Front => SteeringAdjustment=Straight, SpeedAdjustment=Medium (1)"
    "Distance==Medium & Angle==Right => SteeringAdjustment=Straight, SpeedAdjustment=Medium (1)"
    
    % Far distance rules (full speed ahead)
    "Distance==Far => SteeringAdjustment=Straight, SpeedAdjustment=Fast (1)"
];

fis = addRule(fis, rules);

%% Step 5: Visualize the Fuzzy System
% Plot membership functions
figure;
subplot(2,1,1);
plotmf(fis, 'input', 1);
title('Distance to Obstacle (cm)');
subplot(2,1,2);
plotmf(fis, 'input', 2);
title('Angle to Obstacle (degrees)');

figure;
subplot(2,1,1);
plotmf(fis, 'output', 1);
title('Steering Adjustment (degrees)');
subplot(2,1,2);
plotmf(fis, 'output', 2);
title('Speed Adjustment (% of max)');

% Plot rule surfaces
figure;
gensurf(fis, [1 2], 1); % Steering adjustment surface
title('Steering Adjustment Control Surface');
xlabel('Distance (cm)'); ylabel('Angle (degrees)'); zlabel('Steering (degrees)');

figure;
gensurf(fis, [1 2], 2); % Speed adjustment surface
title('Speed Adjustment Control Surface');
xlabel('Distance (cm)'); ylabel('Angle (degrees)'); zlabel('Speed (%)');

%% Step 6: Evaluate the FLC with Sample Inputs
test_cases = [
    10, -30;   % Very close, left
    25, 0;     % Close, front
    60, 45;    % Medium, right
    95, -90;   % Far, left
];

fprintf('\n--- Fuzzy Controller Evaluation ---\n');
for i = 1:size(test_cases, 1)
    input = test_cases(i,:);
    output = evalfis(fis, input);
    
    fprintf('\nCase %d: Distance = %.1f cm, Angle = %.1f°\n', i, input(1), input(2));
    fprintf('  Steering Adjustment: %.1f°\n', output(1));
    fprintf('  Speed Adjustment: %.1f%%\n', output(2));
    
    % Visualize rule firing for this case
    figure;
    plotfis(fis);
    title(sprintf('Rule Activation for Case %d', i));
end

%% Step 7: Save the FIS for Later Use
writeFIS(fis, 'obstacle_avoidance_fis');