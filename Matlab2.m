%% Mamdani Fuzzy Logic Controller for Robot Navigation
%% 1. Fuzzy Inference System Creation
fis = mamfis('Name','ObstacleAvoidance');

% Configure FIS operations
fis.DefuzzificationMethod = 'centroid';  % Crisp output calculation
fis.AndMethod = 'min';                  % For rule antecedents (AND)
fis.OrMethod = 'max';                   % For rule antecedents (OR)
fis.ImplicationMethod = 'min';          % Rule strength application
fis.AggregationMethod = 'max';          % Combine rule outputs

%% 2. Input Variables and Membership Functions

% Input 1: Distance to obstacle (0-200 cm)
fis = addInput(fis, [0 200], 'Name', 'Distance');
% Overlapping trapezoidal and triangular MFs for smooth transitions
fis = addMF(fis, 'Distance', 'trapmf', [0 0 30 50], 'Name', 'VeryClose');
fis = addMF(fis, 'Distance', 'trimf', [40 70 100], 'Name', 'Close');
fis = addMF(fis, 'Distance', 'trimf', [80 120 160], 'Name', 'Medium');
fis = addMF(fis, 'Distance', 'trapmf', [150 170 200 200], 'Name', 'Far');

% Input 2: Angle to obstacle (-180 to 180 degrees)
fis = addInput(fis, [-180 180], 'Name', 'Angle');
% Wide MF for left/right, narrow for front (critical region)
fis = addMF(fis, 'Angle', 'trapmf', [-180 -180 -45 -20], 'Name', 'Left');
fis = addMF(fis, 'Angle', 'trimf', [-30 0 30], 'Name', 'Front');
fis = addMF(fis, 'Angle', 'trapmf', [20 45 180 180], 'Name', 'Right');

%% 3. Output Variables and Membership Functions

% Output 1: Steering adjustment (-45 to 45 degrees)
fis = addOutput(fis, [-45 45], 'Name', 'Steering');
% More MFs near center for finer control
fis = addMF(fis, 'Steering', 'trimf', [-45 -45 -22.5], 'Name', 'HardLeft');
fis = addMF(fis, 'Steering', 'trimf', [-45 -22.5 0], 'Name', 'Left');
fis = addMF(fis, 'Steering', 'trimf', [-22.5 0 22.5], 'Name', 'Straight');
fis = addMF(fis, 'Steering', 'trimf', [0 22.5 45], 'Name', 'Right');
fis = addMF(fis, 'Steering', 'trimf', [22.5 45 45], 'Name', 'HardRight');

% Output 2: Speed adjustment (0-100% of max)
fis = addOutput(fis, [0 100], 'Name', 'Speed');
% Non-linear spacing for safety (more resolution at low speeds)
fis = addMF(fis, 'Speed', 'trapmf', [0 0 10 20], 'Name', 'Stop');
fis = addMF(fis, 'Speed', 'trimf', [15 30 45], 'Name', 'Slow');
fis = addMF(fis, 'Speed', 'trimf', [40 60 80], 'Name', 'Medium');
fis = addMF(fis, 'Speed', 'trapmf', [70 90 100 100], 'Name', 'Fast');

%% 4. Rule Base Design
rules = [
    % Emergency cases (VeryClose distance)
    "Distance==VeryClose & Angle==Left => Steering=HardRight, Speed=Stop (1)"
    "Distance==VeryClose & Angle==Front => Steering=HardLeft, Speed=Stop (1)"
    "Distance==VeryClose & Angle==Right => Steering=HardLeft, Speed=Stop (1)"
    
    % Close obstacle avoidance
    "Distance==Close & Angle==Left => Steering=Right, Speed=Slow (1)"
    "Distance==Close & Angle==Front => Steering=Left, Speed=Slow (1)"
    "Distance==Close & Angle==Right => Steering=Left, Speed=Slow (1)"
    
    % Normal operation
    "Distance==Medium & Angle==Left => Steering=Straight, Speed=Medium (1)"
    "Distance==Medium & Angle==Front => Steering=Straight, Speed=Medium (1)"
    "Distance==Medium & Angle==Right => Steering=Straight, Speed=Medium (1)"
    
    % Clear path
    "Distance==Far => Steering=Straight, Speed=Fast (1)"
    
    % Additional safety rule
    "Distance==VeryClose => Speed=Stop (1)"  % Override for any angle if too close
];

fis = addRule(fis, rules);

%% 5. Visualization
figure;
subplot(2,1,1); plotmf(fis,'input',1); title('Distance MF');
subplot(2,1,2); plotmf(fis,'input',2); title('Angle MF');

figure;
subplot(2,1,1); plotmf(fis,'output',1); title('Steering MF');
subplot(2,1,2); plotmf(fis,'output',2); title('Speed MF');

figure;
gensurf(fis,[1 2],1); title('Steering Control Surface');
xlabel('Distance'); ylabel('Angle'); zlabel('Steering');

figure;
gensurf(fis,[1 2],2); title('Speed Control Surface');
xlabel('Distance'); ylabel('Angle'); zlabel('Speed');

%% 6. Evaluation
test_cases = [
    10, -30;    % Very close left
    60, 0;      % Close front
    120, 45;    % Medium right
    180, -90;   % Far left
];

fprintf('\n=== FLC Evaluation ===\n');
for i = 1:size(test_cases,1)
    in = test_cases(i,:);
    out = evalfis(fis,in);
    fprintf('\nCase %d: Distance=%.1fcm, Angle=%.1f°\n',i,in(1),in(2));
    fprintf('  Steering: %.1f°\n',out(1));
    fprintf('  Speed: %.1f%%\n',out(2));
end

%% 7. Save System
writeFIS(fis,'robot_navigation_fis');