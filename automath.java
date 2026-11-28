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
private DcMotor xdom=null;
private DcMotor ydom=null;
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
double powerB=0.0;
double powerA=0.0;
double powerC=0.0;
double tauA=0.0;
double kpodom=0.0;
double kpodomB=0.0;
double kpodomC=0.0;
double kpodomD=0.0;
double kvB=0.0;
double ksB=0.0;
double kpposition=0.0;
double kpvelocity=0.0;
double kppositionB=0.0;
double kpvelocityB=0.0;
double tauB=0.0;
double kppositionC=0.0;
double kpvelocityC=0.0;
double targetpos=1.2*TICKS_PER_METER;
double tauC=0.0;
double kppositionD=0.0;
double kpvelocityD=0.0;
double tauD=0.0;
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


xdom=hardwareMap.get(DcMotor.class, "xdom");
ydom=hardwareMap.get(DcMotor.class, "ydom");  
leftDriveA = hardwareMap.get(DcMotor.class, "leftDriveA");
rightDriveA = hardwareMap.get(DcMotor.class, "rightDriveA");
leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
intake = hardwareMap.get(DcMotor.class, "intake");
launcher = hardwareMap.get(DcMotorEx.class, "launcher");
windmillServo = hardwareMap.get(CRServo.class, "windmill");
leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");


xdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
ydom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
xdom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
ydom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
long currentticksxodom=xdom.getCurrentPosition();
long currentticksyodom=ydom.getCurrentPosition();
long RcurrentTicksA = rightDriveA.getCurrentPosition();
double deltaTicksR = targetpos-RcurrentTicks;
double deltaTicksL = targetpos-LcurrentTicks;
double deltaTicksRA = targetpos-RcurrentTicksA;
double deltaTicksLA = targetpos-LcurrentTicksA;
double deltaTicksxodom = currentticksxodom - prevticksxodom;
double deltaTicksyodom = currentticksyodom - prevticksyodom;
double deltaTime = currentTime - prevTime;
double manualVelocityL = deltaTicksL / deltaTime; // Ticks per second
double manualVelocityR = deltaTicksR / deltaTime; // Ticks per second
double manualVelocityLA = deltaTicksLA / deltaTime; // Ticks per second
double manualVelocityRA = deltaTicksRA / deltaTime; // Ticks per second
double manualVelocityX = deltaTicksxodom / deltaTime; // Ticks per second
double manualVelocityY= deltaTicksyodom / deltaTime; // Ticks per second


if(currentTime<5){
//rightIntakeServo.setPower(1);
//launch();
}
rightIntakeServo.setPower(0);
launcher.setPower(0);
tauA=func(deltaTicksL,2.0/3.0,1.0,1.5);
kpposition=getkpvelandkppos(0.67,tauA,0);
kpvelocity=getkpvelandkppos(0.67,tauA,1);
kpodom=(kpvelocity/4)-kpposition;
tauB=func(deltaTicksR,2.0/3.0,1.0,1.5);
kppositionB=getkpvelandkppos(0.67,tauB,0);
kpvelocityB=getkpvelandkppos(0.67,tauB,1);
tauC=func(deltaTicksLA,2.0/3.0,1.0,1.5);
kppositionC=getkpvelandkppos(0.67,tauC,0);
kpvelocityC=getkpvelandkppos(0.67,tauC,1);
kpodomC=(kpvelocityC/4)-kppositionC;
tauD=func(deltaTicksRA,2.0/3.0,1.0,1.5);
kppositionD=getkpvelandkppos(0.67,tauD,0);
kpvelocityD=getkpvelandkppos(0.67,tauD,1);
kpodomD=(kpvelocityD/4)-kppositionD;

double pidoutput=uk(kpposition,deltaTicksL,manualVelocityL,kpvelocity,0.67,1.0,0.0,0.07,1.00,kpodom,deltaTicksxodom);
double pidoutputB=uk(kppositionB,deltaTicksR,manualVelocityR,kpvelocityB,0.67,1.0,0.0,0.07,1.00,kpodomB,deltaTicksxodom);
double pidoutputC=uk(kppositionC,deltaTicksLA,manualVelocityLA,kpvelocityC,0.67,1.0,0.0,0.07,1.00,kpodomC,deltaTicksxodom);
double pidoutputD=uk(kppositionD,deltaTicksRA,manualVelocityRA,kpvelocityD,0.67,1.0,0.0,0.07,1.00,kpodomD,deltaTicksxodom);

double power = Math.max(-0.3, Math.min(0.3, pidoutput / bv));  
double powerB = Math.max(-0.3, Math.min(0.3, pidoutputB / bv));  
double powerA = Math.max(-0.3, Math.min(0.3, pidoutputC / bv));  
double powerC = Math.max(-0.3, Math.min(0.3, pidoutputD / bv));
  if(currentTime>5 && currentTime<10){
    leftDrive.setPower(power);
    leftDriveA.setPower(powerA);
    rightDrive.setPower(powerB);
    rightDriveA.setPower(powerC);
  }else{
  leftDrive.setPower(0);
  leftDriveA.setPower(0);
  rightDrive.setPower(0);
  rightDriveA.setPower(0);
  }
  prevTicksL = LcurrentTicks;
  prevTicksR = RcurrentTicks;
  prevTicksLA = LcurrentTicksA;
  prevTicksRA = RcurrentTicksA;
  prevticksyodom = currentticksyodom;
  prevticksxodom = currentticksxodom;
  prevTime=currentTime;
  telemetry.addData("front motor right", RcurrentTicksA);
  telemetry.addData("back motor left", LcurrentTicks);
  telemetry.addData("front motor left", LcurrentTicksA);
  telemetry.addData("back motors right", RcurrentTicks);
  telemetry.update();
  
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
    double rad_to_degrees=180/Math.PI;
    return rad*rad_to_degrees;
}
double degreesToTurnTicks(double degrees) {
double TICKS_PER_METER = TICKS_PER_REV / WHEEL_CIRCUMFERENCE_M; // ~1782.88 ticks/m
double meters = degreesToTurnMeters(degrees);
return Math.round(meters * TICKS_PER_METER);
}
double func(double errorA,double mindistance,double mvel,double xaccel){
  double yb=(Math.abs(errorA)/mvel)+(mvel/xaccel);
  double ya=2*Math.sqrt(Math.abs(errorA)/xaccel);
  if(Math.abs(errorA)>mindistance){
    return yb;
  }else if (Math.abs(errorA)<=mindistance){
    return ya;
  }else{
    return 0;
  }  
}
double getkpvelandkppos(double Acoeff,double inputd,double choice){
  if(inputd==0){
    return 0;
  }
  if(choice==0){
    return 4.0/Math.max(inputd,5.0);
  }else{
    return Acoeff/(Math.max(inputd/16,4.0));
}

}
double uk(double kppos,double epos,double errorvelocity,double kpvel,double ka,double atarget,double vtarget,double ks,double kv,double kpx,double errorx){
  double direction=Math.tanh(epos/0.02);
  double m=10*(537.7/(96*Math.PI));
  double u=kppos*epos+kpvel*errorvelocity+ks*direction+kv*vtarget+ka*atarget+kpx*errorx;
  return u;
}
}
