// 1. Physics-derived constants (bare-motor radian units)
final double Kp = 0.2104;
final double Kd = 0.0235;
final double A  = 0.002104;
final double B  = 0.0186;
final double kS = 0.698; // 96 mm grip wheels on FTC foam tiles

// 2. Unit conversion constants (96 mm wheel, 19.2:1 motor, 537.7 ticks/rev)
final double METERS_TO_MOTOR_RAD = 19.2 / 0.048; // Exactly 400.0 rad/m
final double TICKS_TO_MOTOR_RAD  = (2.0 * Math.PI) / (537.7 / 19.2);

// 3. Example target inputs in METERS (replace with your motion profile or setpoint)
double targetMeters    = 1.5; // Drive 1.5 meters
double targetVelMps    = 0.0; // m/s (0.0 if driving to a stationary point)
double targetAccelMps2 = 0.0; // m/s^2 (0.0 if driving to a stationary point)

// Convert meter targets to bare-motor radians
double targetThetaM     = targetMeters * METERS_TO_MOTOR_RAD;
double targetThetaDotM  = targetVelMps * METERS_TO_MOTOR_RAD;
double targetThetaDDotM = targetAccelMps2 * METERS_TO_MOTOR_RAD;

// 4. Read current state from DcMotorEx encoder
double thetaM    = motor.getCurrentPosition() * TICKS_TO_MOTOR_RAD; // rad
double thetaDotM = motor.getVelocity() * TICKS_TO_MOTOR_RAD;        // rad/s

// 5. Calculate error and total voltage
double error    = targetThetaM - thetaM;
double errorDot = targetThetaDotM - thetaDotM;

// Apply kS in direction of motion (or error if stationary target, outside a small 0.01 m deadband)
double direction = (targetThetaDotM != 0) ? Math.signum(targetThetaDotM)
                 : (Math.abs(error) > (0.01 * METERS_TO_MOTOR_RAD) ? Math.signum(error) : 0.0);

double vFeedback    = (Kp * error) + (Kd * errorDot);
double vFeedforward = (A * targetThetaDDotM) + (B * targetThetaDotM) + (kS * direction);
double vTotal       = vFeedback + vFeedforward;

// 6. Normalize to [-1.0, 1.0] motor power using live battery voltage
double batteryVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
double power = Math.max(-1.0, Math.min(1.0, vTotal / batteryVoltage));

motor.setPower(power);
