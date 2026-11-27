package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="shootandpark", group="StarterBot")
//@Disabled
public class shootandpark extends OpMode
{
private ElapsedTime timer= new ElapsedTime();
public final int LAUNCHER_TARGET_VELOCITY = 1250;
public final int LAUNCHER_MIN_VELOCITY = 1200;
private DcMotor leftDrive = null;
private DcMotor leftDriveA=null;
private DcMotor rightDriveA=null;
private DcMotor rightDrive = null;
private DcMotorEx launcher = null;
private DcMotor intake = null;
private CRServo leftIntakeServo = null;
private CRServo rightIntakeServo = null;
private CRServo windmillServo = null;

static final double WHEEL_DIAMETER_M = 0.096; // 96 mm goBILDA wheels
static final double TRACK_WIDTH_M = 0.398; // ~398 mm center-to-center wheel width
static final double TURNING_RADIUS_M = TRACK_WIDTH_M / 2.0; // r = 0.199 m

// --- MOTOR CONSTANTS (312 RPM Yellow Jacket) ---
static final double TICKS_PER_REV = 537.7;

static final double WHEEL_CIRCUMFERENCE_M = Math.PI * WHEEL_DIAMETER_M;
static final double TICKS_PER_METER = TICKS_PER_REV / WHEEL_CIRCUMFERENCE_M; // ~1782.88 ticks/m

// --- SCRUB TUNING MULTIPLIER ---
// Start at 1.0. If the bot turns 320 deg when commanded 360 deg, set to (360.0 / 320.0) = 1.125
static final double SCRUB_FACTOR = 1.0;
// 2. Unit conversion constants (96 mm wheel, 19.2:1 motor, 537.7 ticks/rev)
final double METERS_TO_MOTOR_RAD = 19.2 / 0.048; // Exactly 400.0 rad/m
final double TICKS_TO_MOTOR_RAD = (2.0 * Math.PI) / (537.7 / 19.2);

// 3. Example target inputs in METERS (replace with your motion profile or setpoint)

// Convert meter targets to bare-motor radians

double motorpowerL=0.0;
double motorpowerR=0.0;
double motorpowerLA=0.0;
double motorpowerRA=0.0;
double intakePower=0.0;
double prevTicksL=0.0;
double prevTicksR=0.0;
double prevTicksLA=0.0;
double prevTicksRA=0.0;
double prevticksxodom=0.0;
double prevticksyodom=0.0;
double prevTime=0.0;
double power=0.0;
//start of moving


// Apply kS in direction of motion (or error if stationary target, outside a small 0.01 m deadband)
// Define a tiny threshold to account for floating-point inaccuracies




/*
* Initialize the hardware variables. Note that the strings used here as parameters
* to 'get' must correspond to the names assigned during the robot configuration
* step.
*/
@Override
public void init() {

/*
* Initialize the hardware variables. Note that the strings used here as parameters
* to 'get' must correspond to the names assigned during the robot configuration
* step.
*/
*/
xodom=hardwareMap.get(DcMotor.class, "xodom");
yodom=hardwareMap.get(DcMotor.class, "yodom");  
leftDriveA = hardwareMap.get(DcMotor.class, "leftDriveA");
rightDriveA = hardwareMap.get(DcMotor.class, "rightDriveA");
leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
intake = hardwareMap.get(DcMotor.class, "intake");
launcher = hardwareMap.get(DcMotorEx.class, "launcher");
windmillServo = hardwareMap.get(CRServo.class, "windmill");
leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");


xodom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
yodom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

leftDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
rightDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
/*
* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
* slow down much faster when it is coasting. This creates a much more controllable
* drivetrain. As the robot stops much quicker.
*/


/*
* Here we set our launcher to the RUN_USING_ENCODER runmode.
* If you notice that you have no control over the velocity of the motor, it just jumps
* right to a number much higher than your set point, make sure that your encoders are plugged
* into the port right beside the motor itself. And that the motors polarity is consistent
* through any wiring.
*/

leftDrive.setDirection(DcMotor.Direction.REVERSE);
rightDrive.setDirection(DcMotor.Direction.FORWARD);
xodom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
yodom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
leftDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
rightDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

/*
* set Feeders to an initial value to initialize the servo controller
*/
leftIntakeServo.setPower(0);
rightIntakeServo.setPower(0);
windmillServo.setPower(0);

/*
* Much like our drivetrain motors, we set the right intake servo to reverse so that both
* servos work to pull elements into the intake.
*/
rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

/*
* Tell the driver that initialization is complete.
*/
telemetry.addData("Status", "Initialized");
}

/*
* Code to run REPEATEDLY after the driver hits INIT, but before they hit START
*/
@Override
public void init_loop() {
}

/*
* Code to run ONCE when the driver hits START
*/
@Override
public void start() {
}
@Override
public void loop() {
double bv = hardwareMap.voltageSensor.iterator().next().getVoltage();
long LcurrentTicks = leftDrive.getCurrentPosition();
double currentTime = timer.seconds();
long RcurrentTicks = rightDrive.getCurrentPosition();
long LcurrentTicksA = leftDriveA.getCurrentPosition();
long currentticksxodom=xodom.getCurrentPosition();
long currentticksyodom=yodom.getCurrentPosition();
long RcurrentTicksA = rightDriveA.getCurrentPosition();
double deltaTicksR = RcurrentTicks - prevTicksR;
double deltaTicksL = LcurrentTicks - prevTicksL;
double deltaTicksRA = RcurrentTicksA - prevTicksRA;
double deltaTicksLA = LcurrentTicksA - prevTicksLA;
double deltaTicksxodom = currentticksxodom - prevticksxodom;
double deltaTicksyodom = currentticksyodom - prevticksyodom;
double deltaTime = currentTime - prevTime;
double manualVelocityL = deltaTicksL / deltaTime; // Ticks per second
double manualVelocityR = deltaTicksR / deltaTime; // Ticks per second
double manualVelocityLA = deltaTicksLA / deltaTime; // Ticks per second
double manualVelocityRA = deltaTicksRA / deltaTime; // Ticks per second
double manualVelocityX = deltaTicksxodom / deltaTime; // Ticks per second
double manualVelocityY= deltaTicksyodom / deltaTime; // Ticks per second
prevTicksL = LcurrentTicks;
prevTicksR = RcurrentTicks;
prevTicksLA = LcurrentTicksA;
prevTicksRA = RcurrentTicksA;
prevticksyodom = currentticksyodom;
prevticksxodom = currentticksxodom;
if(currentTime<5){
rightIntakeServo.setPower(1);
launch();
}
  
double power = Math.max(-1.0, Math.min(1.0, pidoutput / bv));  

}
@Override
public void stop() {
}
void launch() {
rightIntakeServo.setPower(1);
launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY)
{
intakePower += 0.5;
} else {
windmillServo.setPower(0);
}
}
double degreesToTurnMeters(double degrees) {
double WHEEL_DIAMETER_M = 0.096; // 96 mm goBILDA wheels
double TRACK_WIDTH_M = 0.398; // ~398 mm center-to-center wheel width
double TURNING_RADIUS_M = TRACK_WIDTH_M / 2.0; // r = 0.199 m

// --- MOTOR CONSTANTS (312 RPM Yellow Jacket) ---
double TICKS_PER_REV = 537.7;
double WHEEL_CIRCUMFERENCE_M = Math.PI * WHEEL_DIAMETER_M;


// --- SCRUB TUNING MULTIPLIER ---
// Start at 1.0. If the bot turns 320 deg when commanded 360 deg, set to (360.0 / 320.0) = 1.125
double SCRUB_FACTOR = 1.0;
double thetaRadians = Math.toRadians(degrees);
double arcDistanceMeters = TURNING_RADIUS_M * thetaRadians;
return arcDistanceMeters * SCRUB_FACTOR;
}
double radtodegrees(double rad){
    double rad_to_degrees=180/Math.PI
    return rad*rad_to_degrees
}
double degreesToTurnTicks(double degrees) {
double TICKS_PER_METER = TICKS_PER_REV / WHEEL_CIRCUMFERENCE_M; // ~1782.88 ticks/m
double meters = degreesToTurnMeters(degrees);
return Math.round(meters * TICKS_PER_METER);
}
double func(double errorA,double mindistance,double mvel,double xaccel){
  yb=((Math.abs(errorA)/mvel)+(mvel/xaccel)));
  ya=2*Math.sqrt(Math.abs(errorA)/xaccel);
  if(Math.abs(errorA)>mindistance){
    return yb;
  }else if (Math.abs(errorA)<=mindistance){
    return ya;
    
}
double getkpvelandkppos(double Acoeff,double input,double choice){
  if(input!=0){
    return 0
  }
  if(choice==0){
    return 4/input;
  }else if(choice==1){
    return Acoeff/(Math.max(input/16,0.1));
  }
double makepdfeedfoward(double kppos,double errorpos,double errorvelocity,double kpvel,double ka,double atarget,double vtarget,double ks,double kv,double kpx,double kpy,double errorx,double errory){
  double direction=0.0;
  double m=10*(537.7/(96*Math.pi))
  if (Math.abs(vtarget) > 1e-4) {
                direction = Math.signum(vtarget);
  } else if (Math.abs(errrorpos) > m) {
                direction = Math.signum(errorpos);
  } 
  double u=kppos*errorpos+kpvel*errorvelocity+ks*direction+kv*vtarget+ka*atarget+kpx*errorx+kpy*errory;
  return u;
}
double divide(double n,double d){
  return n/d; 
}
}
}

}
}
