
///just care about mecanum wheel code
//add high pass filter to wheels
//find dampening frequency t=1/((1+2pi)*fc*deltaT)
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.io.Serializable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@TeleOp (name = "shakyhands_teleop (Blocks to Java)")
public class shakyhands_teleop extends LinearOpMode {
  double fixedtheta=65;//find this
  public double radwheelBig=48; //mm wheel radius
  public double radwheelSmall=24; //mm wheel radius 
  public double radwheelGecko=36; //mm wheel radius
  public double BIGgear=100;//teeth
  public double beltteeth=46;//teeth
  public double toothsprocket=10;//teeth
  public double smallgear=20;//teeth
  public double midgear=30;//teeth
  double alpha=0.2;//tune
  public double Bigratio=50.9://ratio
  public double Midratio=19.2://ratio
  public double smallratio=1/10;//ratio
  public double intakeRollsrad=8;//mm
  double rotater=0;
  double rotaterA=0;
  double motifs=0;
  double deltaX=0;
  double startx=0;
  double runtime=0;
  double starty=0;
  double deltaY=0;
  boolean USE_WEBCAM;
  AprilTagProcessor myAprilTagProcessor;
  Position cameraPosition;
  YawPitchRollAngles cameraOrientation;
  VisionPortal myVisionPortal;
  double rangeB=0;
  public double offsetX=9.129397076417323;
  public double offsetY=5.1496063;//fine tune this
  double pointAx=-16.5354;//tune this in inches times -1
  double pointBx=0.0001;
  double pointCx=0;
  double pointAy=0.0001;
  double pointBy=16.5354;// robot height of camera in inches
  double pointCy=38.759843+offsetY;

  double currentpositionY=0;
  double currentpositionX=0;
  double gravity=386.08858267717;
  boolean thisExpUp;
  boolean thisExpDn;
  boolean thisGainUp;
  boolean thisGainDn;
  boolean lastExpUp;
  boolean lastExpDn;
  boolean lastGainUp;
  boolean lastGainDn;
  double onerev=383.6;
  double desiredspeed=0;
  double intialspeed=0; //intial speed before change measured in ticks adjust value
  private DcMotor backleft;
  private DcMotor backright;
  private DcMotor frontright;
  private DcMotor frontleft;
  private Servo pivotintake;
  private Servo pivotintakeA;
  private Servo shooterholder;
  private Servo artifactholder;
  private CRServo belt;
  private CRServo beltA;
  private DcMotor intake;
  private DcMotor shooterwheelA;
  private DcMotor shooterwheelB;
  private CRServo holder;
  private DcMotor X;
 
  public double change=10; //adjust value
  public double degree1=0.5;
  public double degree2=0;
  public double latchopen=0.5;
  public double latchclose=0;
  public double artifactholderopen=0.5; // adjust value in the future
  public double artifactholderclose=0; // adjust value in the future
  public double shooterholderopen=0.5; // adjust value in the future
  public double shooterholderclose=0; // adjust value in the future
  public double beltspeed1=-1; // adjust value in the future
  public double beltspeed2=1; // adjust value in the future
  public double Circumference=76.8*Math.PI; //in mm
 
  double[] motif={0,0,0,0};
  double[] correctmotif={0,0,0,0};
  double red=0;
  double green=0;
  double blue=0;
  double test=0;
  double sense=0;
  double Xa=0;
  double Ya=0;
  double y=0;
  double x=0;
  double Za=0;
  double Pitch=0;
  double Yaw=0;
  double Roll=0;
  double Ycenter=0;
  double Xcenter=0;
  int i=0;
  double A=0;
  double B=0;
  double C=0;

  public double kSB=0;//find this value
  public double kAB=0;//find this value
  public double kVB=0;//find this value
  double DesiredVB=0;//find this value
  double DesiredAB=0;//find this value
     
  public double feedforwardtermB(double DesiredVB,double DesiredAB,double kSB,double kVB,double kAB){
           this.kSB=kSB;
           this.kVB=kVB;
           this.kAB=kAB;
           double outputffB=kSB*sign(DesiredVB)+kVB*DesiredVB+kAB*DesiredAB;
           return outputffB;
  }
  public double sign(double v){
         if(v>0){
             return 1;
         }
         if(v==0){
             return 0;
         }
         else{
            return -1;
         }
  }

  public double kpshooterB; //find this value
  public double kishooterB; //find this value
  public double kdshooterB;//find this value
  double previousErrorshooterB=0;
  double intergralshooterB=0; //assign a value in the future to intergral
  double minOutputshooterB=0; //assign a value in the future to minoutput
  double maxOutputshooterB=0; //assign a value in the future to maxoutput

  public double PIDshooterB(double kpshooterB, double kishooterB, double kdshooterB, double shooterB1, double shooterB2, double shooterB3){
   
         this.kpshooterB=kpshooterB;
         this.kishooterB=kishooterB;
         this.kdshooterB=kdshooterB;
         double outputshooterBa = kpshooterB * shooterB1 + kishooterB * shooterB3 + kdshooterB * shooterB2;
         return outputshooterBa;
  }
  public double calcshooterB(double targetshooterB,double currentshooterB){
          double errorshooterB = targetshooterB - currentshooterB;
          double integralmaxB=1000;//update this value if needed
          double timeBs=0.2; //update if needed
          double integralshooterB =+ errorshooterB*timeBs;
          if(integralshooterB>integralmaxB){
              integralshooterB=integralmaxB;
          }
          if(integralshooterB<(integralmaxB*-1)){
              integralshooterB=integralmaxB*-1;
          }
          double derivativeshooterB = errorshooterB - previousErrorshooterB/timeBs;
          double outputshooterBa = PIDshooterB(kpshooterB, kishooterB, kdshooterB, errorshooterB, derivativeshooterB, integralshooterB);
          double outputshooterB = Math.max(minOutputshooterB, Math.min(maxOutputshooterB, outputshooterBa));
 
          double previousErrorshooterB = errorshooterB;
          return outputshooterB;
  }
  public void resetshooterB(){
         double previousErrorshooterB=0;
         double intergralshooterB=0;
  }

 

  public double answervelocity(double heightofgoal, double g, double theta){
       return (Math.sqrt(2*g*heightofgoal)/Math.sin(theta));
  }
  @Override
  public void runOpMode() {
 
    ElapsedTime runtime = new ElapsedTime();
    pivotintake = hardwareMap.get(Servo.class, "pivotintake");
    pivotintakeA = hardwareMap.get(Servo.class, "pivotintakeA");
    belt = hardwareMap.get(CRServo.class, "belt");
    beltA = hardwareMap.get(CRServo.class, "beltA");

 
    double speedOfintakeOff=0;
    double speedOfintakeOn=1;
    double yAxis= -gamepad2.left_stick_y;
    double xAxis= gamepad2.left_stick_x;
    double zAxis= gamepad1.right_stick_y;
    double deadzone=0.2;
    double x=0;
    double y=0;
    double turn=0;
    double backleft_A;
    double swerve_A;
    double swerve_B;
    double backright_A;
    double frontleft_A;
    double frontright_A;
    shooterholder= hardwareMap.get(Servo.class, "shooterholder");
    artifactholder= hardwareMap.get(Servo.class, "artifactholder");
    shooterwheelA = hardwareMap.get(DcMotor.class, "shooterwheelA");
    shooterwheelB = hardwareMap.get(DcMotor.class, "shooterwheelB");
    holder = hardwareMap.get(CRServo.class, "holder");
    X=hardwareMap.get(DcMotor.class, "X");
    intake = hardwareMap.get(DcMotor.class, "intake");
    backleft = hardwareMap.get(DcMotor.class, "backleft");
    backright = hardwareMap.get(DcMotor.class, "backright");
    frontright = hardwareMap.get(DcMotor.class, "frontright");
    frontleft = hardwareMap.get(DcMotor.class, "frontleft");  
    backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    X.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    shooterwheelA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    shooterwheelB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    shooterwheelB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    shooterwheelA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    X.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    // Initialize AprilTag before waitForStart.

    // Wait for the match to begin.
    telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
    telemetry.addData(">", "Touch START to start OpMode");
    telemetry.update();
    runtime.reset();
    waitForStart();
    //runtime.reset();
    while (opModeIsActive()) {
      double volts=hardwareMap.voltageSensor.iterator().next().getVoltage();
      y = gamepad2.left_stick_y;
      x = gamepad2.left_stick_x;
      if(Math.abs(y)<deadzone){
        y=0;
      }else{
        y=Math.pow(y,3);
      }
      if(Math.abs(x)<deadzone){
        x=0;      
      }else{
        x=Math.pow(x,3);
      }  
     
      StartA=frontright.getCurrentPosition();
      StartB=frontleft.getCurrentPosition();
      StartC=backright.getCurrentPosition();
      StartD=backleft.getCurrentPosition();
      turn = gamepad2.right_stick_x;
      rotater= gamepad1.left_stick_y;
      rotaterA= gamepad1.right_stick_y;
      swerve_A=-turn;
      swerve_B=turn;
      frontleft_A = y - x ;
      frontright_A = (y + x) ;
      backleft_A = (y + x) ;
      backright_A = (y - x) ;
     
      if (gamepad2.right_bumper) {
          double precision=1-gamepad2.left_trigger;    
          swerve_A=precision*(swerve_A);
          swerve_B=precision*(swerve_B);
          frontleft_A=precision*(frontleft_A);
          frontright_A=precision*(frontright_A);
          backleft_A=precision*(backleft_A);
          backright_A=precision*(backright_A);
      }

      if (gamepad2.left_bumper) {
          double precision=0.3-(gamepad2.left_trigger*0.3);
          swerve_A=precision*(swerve_A);
          swerve_B=precision*(swerve_B);
          frontleft_A=precision*(frontleft_A);
          frontright_A=precision*(frontright_A);
          backleft_A=precision*(backleft_A);
          backright_A=precision*(backright_A);
      }
      if(!gamepad2.left_bumper && !gamepad2.right_bumper){
          double precision=0.6-(gamepad2.left_trigger*0.6);                  
          swerve_A=precision*(swerve_A);
          swerve_B=precision*(swerve_B);                                          
          frontleft_A=precision*(frontleft_A);
          frontright_A=precision*(frontright_A);
          backleft_A=precision*(backleft_A);
          backright_A=precision*(backright_A);
      }
                 

      if(gamepad2.a){
              frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
              frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
              backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
              backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      }
      else if(gamepad2.b){
              frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);    
              frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
              backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);    
              backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      }
      if(turn==0 && volts>10.0){
              frontright.setPower(-frontright_A);        
              frontleft.setPower(-frontleft_A);          
              backleft.setPower(backleft_A);            
              backright.setPower(backright_A);
      }else if(volts<10.0 && turn==0){
        double rawA=frontright.getCurrentPosition();
        double rawB=frontleft.getCurrentPosition();
        double rawC=backright.getCurrentPosition();
        double rawD=backleft.getCurrentPosition();
      }
      if(turn!=0 && volts>10.0){
              frontright.setPower(-swerve_A);        
              frontleft.setPower(-swerve_A);          
              backleft.setPower(swerve_B);            
              backright.setPower(swerve_B);
      }else if(volts<10.0 && turn!=0){
        double rawA=frontright.getCurrentPosition();
        double rawB=frontleft.getCurrentPosition();
        double rawC=backright.getCurrentPosition();
        double rawD=backleft.getCurrentPosition();
       
      }
      if(gamepad1.right_trigger>0){
         beltA.setPower(0);
         belt.setPower(0);
      }
      //if(gamepad1.y){
      //   artifactholder.setPosition(artifactholderopen);
      //}
      //if(gamepad1.x){
      //   artifactholder.setPosition(artifactholderclose);
      //}
      //if(gamepad1.dpad_up){
      //   shooterholder.setPosition(shooterholderclose);
      //}
      //if(gamepad1.dpad_down){
      //   shooterholder.setPosition(shooterholderopen);
      //}    
      //if(gamepad1.dpad_left){
      //  pivotintakeA.setPosition(latchopen);
      //}
      //if(gamepad1.dpad_right){
      //  pivotintakeA.setPosition(latchclose);
      //}
      //if(gamepad1.a){
      //  pivotintake.setPosition(degree1);
      //}
      //if(gamepad1.b){
      //  pivotintake.setPosition(degree2);
      //}

      if(gamepad1.left_bumper){
           intake.setPower(-speedOfintakeOn);
           holder.setPower(speedOfintakeOn);
           
      }
      if(gamepad1.right_bumper){
           intake.setPower(speedOfintakeOn);
            holder.setPower(-speedOfintakeOn);
      }
      else{
       
          intake.setPower(speedOfintakeOff);
          holder.setPower(speedOfintakeOff);

      }
      if(gamepad1.a){
         
         



       
          double power=1-gamepad1.left_trigger;
          shooterwheelA.setPower(-power);
          shooterwheelB.setPower(power);
      }
      if(gamepad1.b){
        shooterwheelA.setPower(0);
        shooterwheelB.setPower(0);
      }
      if(gamepad1.x){
        beltA.setPower(-beltspeed1);
        belt.setPower(beltspeed1);
      }
      if(gamepad1.y){
        beltA.setPower(-beltspeed2);
        belt.setPower(beltspeed2);
      }
      //beltA.setPower(0);
      //belt.setPower(0);
     


    }  
  }
}
