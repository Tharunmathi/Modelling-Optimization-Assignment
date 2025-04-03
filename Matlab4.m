% Define Fuzzy Inference System (FIS)
fan = newfis('OptimizationFLC', 'mamdani'); % Create Mamdani fuzzy system

% Step 1: Define input variable (fitness)
fan = addvar(fan, 'input', 'Fitness', [0 100]); % Example fitness range for optimization

% Step 2: Define membership functions for input variable
fan = addmf(fan, 'input', 1, 'Low', 'trimf', [0 0 20]); % Low fitness value
fan = addmf(fan, 'input', 1, 'Medium', 'trimf', [20 50 80]); % Medium fitness value
fan = addmf(fan, 'input', 1, 'High', 'trimf', [80 100 100]); % High fitness value

% Step 3: Define output variable (control action)
fan = addvar(fan, 'output', 'ControlAction', [0 10]); % Example control output range

% Step 4: Define membership functions for output variable
fan = addmf(fan, 'output', 1, 'Low', 'trimf', [0 0 2]); 
fan = addmf(fan, 'output', 1, 'Medium', 'trimf', [2 5 8]); 
fan = addmf(fan, 'output', 1, 'High', 'trimf', [8 10 10]); 

% Step 5: Define fuzzy rules for controlling the optimization process
rules = [
    1 1 1 1;   % Low fitness -> Low control action
    2 2 2 1;   % Medium fitness -> Medium control action
    3 3 3 1;   % High fitness -> High control action
];

% Add rules to the fuzzy system
fan = addrule(fan, rules);

% Step 6: Function evaluation (fitness calculation) for the three functions
fitnessResults = zeros(3, 1); % Store fitness results for each function
controlActions = zeros(3, 1); % Store control actions based on fitness

% Define the functions to be compared
functions = {'Shifted Sphere Function', 'Shifted Rosenbrock''s Function', 'Shifted Rastrigin''s Function'};

% Example positions for each function (can be optimized using FLC)
positions = [1, 2; 0.5, 1.5; -1, 1]; % Random example positions

% Evaluate the fitness for each function
for i = 1:3
    switch functions{i}
        case 'Shifted Sphere Function'
            fitnessResults(i) = shiftedSphere(positions(i,:)); % Fitness calculation
        case 'Shifted Rosenbrock''s Function'
            fitnessResults(i) = shiftedRosenbrock(positions(i,:)); % Fitness calculation
        case 'Shifted Rastrigin''s Function'
            fitnessResults(i) = shiftedRastrigin(positions(i,:)); % Fitness calculation
    end
    
    % Evaluate the fuzzy system to get the control action based on fitness
    controlActions(i) = evalfis(fitnessResults(i), fan); % Get the output control action
end

% Display results
disp('Fitness Results:');
disp(fitnessResults);
disp('Control Actions:');
disp(controlActions);

% --- Function Definitions ---

% Shifted Sphere Function
function result = shiftedSphere(position)
    shiftedPos = position - 1; % Shift the position by 1
    result = sum(shiftedPos.^2); % Sum of squares of the shifted position
end

% Shifted Rosenbrock's Function
function result = shiftedRosenbrock(position)
    shiftedPos = position - 1; % Shift the position by 1
    result = sum(100*(shiftedPos(2:end) - shiftedPos(1:end-1).^2).^2 + (1 - shiftedPos(1:end-1)).^2);
end

% Shifted Rastrigin's Function
function result = shiftedRastrigin(position)
    shiftedPos = position - 1; % Shift the position by 1
    result = 10*length(position) + sum(shiftedPos.^2 - 10*cos(2*pi*shiftedPos)); % Rastrigin formula
end
