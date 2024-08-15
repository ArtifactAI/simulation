% Compute gravity force vector of vehicle aligned with NED frame
mass_kg = 999;
gravity_mps2 = 9.81;
weight_N = mass_kg * gravity_mps2;

massPropertiesBus.aircraftMass_kg = mass_kg;
bodyStateBus.DCM_be = eye(3);
earthEnvironmentBus.gravityConstant_mps2 = gravity_mps2;

verify(isClose(weightInBod_N(1), 0));
verify(isClose(weightInBod_N(2), 0));
verify(isClose(weightInBod_N(3), weight_N));

% Compute gravity force vector when vehicle is pitched upwards

pitch_rad = 30*pi/180
DCM_be =  [cos(pitch_rad) 0 -sin(pitch_rad);
    0 1 0;
    sin(pitch_rad) 0 cos(pitch_rad)];

massPropertiesBus.aircraftMass_kg = mass_kg;
bodyStateBus.DCM_be = DCM_be
earthEnvironmentBus.gravityConstant_mps2 = gravity_mps2;

verify(isClose(weightInBod_N(1), -weight_N * sin(pitch_rad)));
verify(isClose(weightInBod_N(2), 0));
verify(isClose(weightInBod_N(3), weight_N * cos(pitch_rad)));

% Compute gravity force vector when vehicle is pitched, rolled, and yawed

yaw_rad = 20*pi/180;
pitch_rad = 30*pi/180;
roll_rad = -10*pi/180;

Rx = [1 0 0;
    0 cos(roll_rad) sin(roll_rad);
    0 -sin(roll_rad) cos(roll_rad)];

Ry =  [cos(pitch_rad) 0 -sin(pitch_rad);
    0 1 0;
    sin(pitch_rad) 0 cos(pitch_rad)];

Rz = [cos(yaw_rad) sin(yaw_rad) 0;
    -sin(yaw_rad) cos(yaw_rad) 0;
    0 0 1];

DCM_be = Rx*Ry*Rz

massPropertiesBus.aircraftMass_kg = mass_kg;
bodyStateBus.DCM_be = DCM_be
earthEnvironmentBus.gravityConstant_mps2 = gravity_mps2;

verify(isClose(weightInBod_N(1), -weight_N * sin(pitch_rad)));
verify(isClose(weightInBod_N(2), weight_N * sin(roll_rad) * cos(pitch_rad)));
verify(isClose(weightInBod_N(3), weight_N * cos(roll_rad) * cos(pitch_rad)));