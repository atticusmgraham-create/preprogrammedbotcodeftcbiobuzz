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

@Autonomous(name="TheAuto", group="StarterBot")
//@Disabled
public class TheAuto extends OpMode
{        
        private ElapsedTime timer= new ElapsedTime();
        public final int LAUNCHER_TARGET_VELOCITY = 1250;
        public final int LAUNCHER_MIN_VELOCITY = 1200;
        private DcMotor leftDrive = null;
        private DcMotor rightDrive = null;
        private DcMotorEx launcher = null;
        private DcMotor intake = null;
        private CRServo leftIntakeServo = null;
        private CRServo rightIntakeServo = null;
        private CRServo windmillServo = null;
        final double Kp = 0.2104;
        final double Kd = 0.02349;
        final double A  = 0.002104;
        final double B  = 0.0186;
        final double kS = 0.698; // 96 mm grip wheels on FTC foam tiles
        
        // 2. Unit conversion constants (96 mm wheel, 19.2:1 motor, 537.7 ticks/rev)
        final double METERS_TO_MOTOR_RAD = 19.2 / 0.048; // Exactly 400.0 rad/m
        final double TICKS_TO_MOTOR_RAD  = (2.0 * Math.PI) / (537.7 / 19.2);

        // 3. Example target inputs in METERS (replace with your motion profile or setpoint)
        final double targetMeters    = 0.4; // Drive 1.5 meters
        final double targetVelMps    = 0.0; // m/s (0.0 if driving to a stationary point)
        final double targetAccelMps2 = 0.0; // m/s^2 (0.0 if driving to a stationary point)
        final double targetMetersA    = 0.4; // Drive 1.5 meters
        final double targetVelMpsA    = 0.0; // m/s (0.0 if driving to a stationary point)
        final double targetAccelMps2A = 0.0; // m/s^2 (0.0 if driving to a stationary point)
        final double VELOCITY_THRESHOLD = 1e-4; 
        final double POSITION_DEADBAND = 0.01 * METERS_TO_MOTOR_RAD;
        // Convert meter targets to bare-motor radians
        double targetThetaM     = targetMeters * METERS_TO_MOTOR_RAD;
        double targetThetaDotM  = targetVelMps * METERS_TO_MOTOR_RAD;
        double targetThetaDDotM = targetAccelMps2 * METERS_TO_MOTOR_RAD;
        double targetThetaMA     = targetMetersA * METERS_TO_MOTOR_RAD;
        double targetThetaDotMA  = targetVelMpsA * METERS_TO_MOTOR_RAD;
        double targetThetaDDotMA = targetAccelMps2A * METERS_TO_MOTOR_RAD;
        double motorpowerL=0.1;
        double motorpowerR=0.1;
        double intakePower=0.0;
        double prevTicksL=0.0;
        double prevTicksR=0.0;
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
              leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
              rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
              intake = hardwareMap.get(DcMotor.class, "intake");
              launcher = hardwareMap.get(DcMotorEx.class, "launcher");
              windmillServo = hardwareMap.get(CRServo.class, "windmill");
              leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
              rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
      
              /*
               * To drive forward, most robots need the motor on one side to be reversed,
               * because the axles point in opposite directions. Pushing the left stick forward
               * MUST make robot go forward. So adjust these two lines based on your first test drive.
               * Note: The settings here assume direct drive on left and right wheels. Gear
               * Reduction or 90 Deg drives may require direction flips
               */
              leftDrive.setDirection(DcMotor.Direction.FORWARD);
              rightDrive.setDirection(DcMotor.Direction.REVERSE);
              leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
              rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
              leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
              rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            double batteryVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            long LcurrentTicks = -leftDrive.getCurrentPosition();
            double currentTime = timer.seconds();
            long RcurrentTicks = -rightDrive.getCurrentPosition();
            double deltaTicksR = RcurrentTicks - prevTicksR;
            double deltaTicksL = LcurrentTicks - prevTicksL;
            double deltaTime = currentTime - prevTime;
            double manualVelocityL = deltaTicksL / deltaTime; // Ticks per second
            double manualVelocityR = deltaTicksR / deltaTime; // Ticks per second

        // 4. Read current state from DcMotorEx encoder
            double thetaML    = LcurrentTicks * TICKS_TO_MOTOR_RAD; // rad
            double thetaDotML = (manualVelocityL)* TICKS_TO_MOTOR_RAD;        // rad/s
            double thetaMR    = RcurrentTicks * TICKS_TO_MOTOR_RAD; // rad
            double thetaDotMR = (manualVelocityR)* TICKS_TO_MOTOR_RAD;    
            prevTicksL = LcurrentTicks;
            prevTicksR = RcurrentTicks;
            prevTime = currentTime;
        // 5. Calculate error and total voltage
            double errorL    = targetThetaM - thetaML;
            double errorDotL = targetThetaDotM - thetaDotML;
            double errorR    = targetThetaM - thetaMR;
            double errorDotR = targetThetaDotM - thetaDotMR;
            motorpowerL=drive(errorL,Kp,Kd,errorDotL,targetThetaDotM,B,batteryVoltage,A,targetThetaDDotM,kS,VELOCITY_THRESHOLD,POSITION_DEADBAND);
            motorpowerR=drive(errorR,Kp,Kd,errorDotR,targetThetaDotM,B,batteryVoltage,A,targetThetaDDotM,kS,VELOCITY_THRESHOLD,POSITION_DEADBAND);
            
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", motorpowerL, motorpowerR);
            if(currentTime<5){
                
                leftDrive.setPower(-motorpowerL);
                rightDrive.setPower(-motorpowerR);
            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            if(currentTime>5 && currentTime<8){

                batteryVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
                LcurrentTicks = -leftDrive.getCurrentPosition();
                currentTime = timer.seconds();
                RcurrentTicks = -rightDrive.getCurrentPosition();
                deltaTicksR = RcurrentTicks - prevTicksR;
                deltaTicksL = LcurrentTicks - prevTicksL;
                deltaTime = currentTime - prevTime;
                manualVelocityL = deltaTicksL / deltaTime; // Ticks per second
                manualVelocityR = deltaTicksR / deltaTime; // Ticks per second
    
            // 4. Read current state from DcMotorEx encoder
                thetaML    = LcurrentTicks * TICKS_TO_MOTOR_RAD; // rad
                thetaDotML = (manualVelocityL)* TICKS_TO_MOTOR_RAD;        // rad/s
                thetaMR    = RcurrentTicks * TICKS_TO_MOTOR_RAD; // rad
                thetaDotMR = (manualVelocityR)* TICKS_TO_MOTOR_RAD;    
                prevTicksL = LcurrentTicks;
                prevTicksR = RcurrentTicks;
                prevTime = currentTime;
                double errorLA    = targetThetaMA - thetaML;
                double errorDotLA = targetThetaDotMA - thetaDotML;
                double errorRA    = targetThetaMA - thetaMR;
                double errorDotRA = targetThetaDotMA - thetaDotMR;
                motorpowerL=drive(errorLA,Kp,Kd,errorDotLA,targetThetaDotMA,B,batteryVoltage,A,targetThetaDDotMA,kS,VELOCITY_THRESHOLD,POSITION_DEADBAND);
                motorpowerR=drive(errorRA,Kp,Kd,errorDotRA,targetThetaDotMA,B,batteryVoltage,A,targetThetaDDotMA,kS,VELOCITY_THRESHOLD,POSITION_DEADBAND);
                
                leftDrive.setPower(-motorpowerL);
                rightDrive.setPower(motorpowerR);
                
            
              }
              if(currentTime>10 && currentTime<22){
                rightIntakeServo.setPower(1);
                launch();
              }
              if(currentTime>22){
                donothing();  
              }
            } 
          @Override
          public void stop() {
          }
          void launch() {
            
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY)
            {
                intakePower += 0.5;
            } else {
                windmillServo.setPower(0);
            }
          }
          double move(double n){
              if(Math.round(n)==0){
                 return 0;    
              }else{
                  return n;
              }
          }
          double drive(double er,double h,double d,double ed,double TTDM,double Bye,double bv,double As,double TTDDM,double kSp,double VT,double PD){
              double direction = 0.0;

              if (Math.abs(TTDM) > VT) {
                direction = Math.signum(TTDM);
              } else if (Math.abs(er) > PD) {
                direction = Math.signum(er);
              }

        
              double vFeedback    = (h * er) + (d * ed);
              double vFeedforward = (As * TTDDM) + (Bye * TTDM) + (kSp * direction);
              double vTotal       = vFeedback + vFeedforward;
        
        // 6. Normalize to [-1.0, 1.0] motor power using live battery voltage
              
              double power = Math.max(-1.0, Math.min(1.0, vTotal / bv));
        
              return power;
          }
          double donothing(){
              launcher.setVelocity(0);
              rightIntakeServo.setPower(0);
              return 0;
          }
}
