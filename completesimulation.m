% Define parameters
one_magnet_magnetic_field = 0.5 ; % (Tesla)
amount_of_magnets = 10 ; % (used_amount_of_magnets)
total_magnetic_field_density = one_magnet_magnetic_field * amount_of_magnets; % Tesla (Assuming each magnet has the same constant magnetic field density)
diameter_of_a_wire_round = 2.0;  % cm
radius = diameter_of_a_wire_round / 2; % cm 
area = pi * radius^2 * 10^-4; % m^2
number_of_turns = 1000; % Number of turns in the coil
coil_resistance = 50.0; % Ohms (Resistance of the coil)
magnet_velocity = 2.0; % m/s (Velocity of the neodymium magnet through the PVC pipe)
length_of_the_coil_wrap_area = 10; % cm
T = (2 * length_of_the_coil_wrap_area*10^-2) / magnet_velocity; % (The amount of time the magnetic field changes)

% The time vector over one second(10HZ)
time = linspace(0,1.00, 25000); % Adjust the number of points as needed

% Calculate the change of magnetic flux through the coil over time
changing_magnetic_flux = total_magnetic_field_density * area / T;

% Calculate the EMF (voltage) induced in the coil using Faraday's law
EMF = -number_of_turns * changing_magnetic_flux;

% Calculate the AC current flowing through the coil (Assuming coil acts as a load)
AC_current = EMF / coil_resistance * sin(2 * pi / T * time);

% Define parameters
% Calculate the peak AC voltage
peak_AC_voltage = max(abs(AC_current)) * coil_resistance ;

% Calculate the frequency
frequency = 1/T ; % Hz
% Load resistance (in ohms)
R_load = 50; 

% Time vector
t = linspace(0,1.00,25000);

% Generate AC voltage waveform (sinusoidal)
AC_voltage = peak_AC_voltage * sin(2 * pi * frequency * t);


% Plot the graph about AC voltage 
figure;
plot(time, AC_voltage, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('AC voltage (V)');
title('Generated AC Voltage over one second ');
grid on;


% Full Bridge Rectifier simulation

DC_voltage = zeros(size(t));
for i = 1:length(t)
    if AC_voltage(i) > 0
        DC_voltage(i) = AC_voltage(i);
    else
        DC_voltage(i) = -AC_voltage(i);
    end
end

% Plot AC and DC voltages
figure;
plot(t, AC_voltage, 'b', 'LineWidth', 2);
hold on;
plot(t, DC_voltage, 'r', 'LineWidth', 2);
hold off;
xlabel('Time (s)');
ylabel('Voltage (V)');
title('AC to DC Conversion using Full Bridge Rectifier');
legend('AC Voltage', 'DC Voltage');

% Calculate the DC current flowing through the load
DC_current = DC_voltage / R_load;

% Plot the DC current
figure;
plot(t, DC_current, 'g', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('DC Current (A)');
title('DC Current after Rectification');

% Define parameters
supercapacitor_capacity = 1.5; % Supercapacitor capacity in Farads
supercapacitor_voltage_maximum = 15; % Maximum voltage of the supercapacitor (in volts)
supply_voltage = 15.7080;% in volts
if supply_voltage > supercapacitor_voltage_maximum
    supply_voltage = supercapacitor_voltage_maximum ;
end 
discharge_voltage = 12.000; % in volts
charging_current = 0.314; % Charging current during charging phase (in Amperes)
discharging_current = 0.800; % Discharging current during discharging phase (in Amperes)

% Time vector
t = linspace(0, 100, 1000); % Adjust the number of points as needed

% Initialize supercapacitor capacitance vector
capacitance_supercapacitor = zeros(size(t));

% Simulation - Charging the supercapacitor
for i = 1:length(t)
    % Calculate the charging of supercapacitor (constant DC current)
    capacitance_supercapacitor(i) = min((charging_current * t(i)) / supply_voltage, supercapacitor_capacity);
end

% Plot supercapacitor charge over time
figure;
plot(t, capacitance_supercapacitor, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Supercapacitor capacitance (F)');
title('Supercapacitor Charging');




% Time vector
t = linspace(0, 100, 1000); % Adjust the number of points as needed

% Initialize supercapacitor capacitance vector
capacitance_supercapacitor = zeros(size(t));


% Simulation - Discharging the supercapacitor
for i = 1:length(t)
    % Calculate the discharging of supercapacitor (constant DC current)
    capacitance_supercapacitor(i) = max(supercapacitor_capacity - ((discharging_current * t(i))/discharge_voltage), 0);
end

% Ensure the supercapacitor is fully discharged at the end
capacitance_supercapacitor(end) = 0;

% Plot supercapacitor discharge over time
figure;
plot(t, capacitance_supercapacitor, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Supercapacitor capacitance (F)');
title('Supercapacitor Discharge');


% Define parameters for LED bulbs
num_series_strings = 4;
num_parallel_bulbs_per_string = 4;
num_total_bulbs = num_series_strings * num_parallel_bulbs_per_string;

LED_resistance = 15; % Ohms (Resistance of each LED bulb)
LED_current_desired = discharging_current; % Desired LED current (in Amperes)

% Initialize vector to store LED status (On/Off)
LED_status = zeros(size(t));

% Loop through time to simulate capacitor discharge
for i = 1:length(t)
    
    % Check if the total current is greater than the desired LED current
    if capacitance_supercapacitor(i) > 0
        % Set LED status to 1 (On)
        LED_status(i) = 1;
    else
        % Set LED status to 0 (Off)
        LED_status(i) = 0;
    end
end

% Plot LED status over time
figure;
stairs(t, LED_status, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('LED Status');
title('LED On/Off Status during Supercapacitor Discharge');
ylim([-0.1, 1.1]); % Set y-axis limits for clarity
grid on;
