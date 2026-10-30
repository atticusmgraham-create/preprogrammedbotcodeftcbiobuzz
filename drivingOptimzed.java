package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "DrivingOptimized", group = "StarterBot")
//@Disabled
public class DrivingOptimized extends OpMode {

    // Hardware (names fixed to camelCase so they match everywhere they're used)
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    // Launcher velocities in encoder ticks per second
    public final int LAUNCHER_TARGET_VELOCITY = 1250;
    public final int LAUNCHER_MIN_VELOCITY = 1200;

    double leftPower = 0;
    double rightPower = 0;
    double y = 0;
    double x = 0;
    final double DEADZONE = 0.2;

    double intakePower = 0;

    @Override
    public void init() {
        // The strings here must match your Robot Configuration exactly
        leftDrive = hardwareMap.get(DcMotor.class, "leftdrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightdrive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmill");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5));

        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        // ---- Drive inputs: deadzone + cubic curve ----
        y = shape(gamepad2.left_stick_y);
        x = shape(gamepad2.right_stick_x);

        // ---- Speed scaling (else-if so bumpers can't stack) ----
        double precision;
        if (gamepad2.right_bumper) {
            precision = 1 - gamepad2.left_trigger;
        } else if (gamepad2.left_bumper) {
            precision = 0.3 * (1 - gamepad2.left_trigger);
        } else {
            precision = 0.6 * (1 - gamepad2.left_trigger);
        }
        y *= precision;
        x *= precision;

        // ---- Brake / float: hold B to coast, otherwise brake ----
        if (gamepad2.b) {
            floatMode();
        } else {
            brakeMode();
        }

        // Always drive every loop. (Before, the motors were left at their
        // last power whenever A or B was held.)
        arcadeDrive(-y, x);

        // ---- Intake ----
        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

        // launch() may modify intakePower, so call it BEFORE writing to hardware
        launch();

        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);

        // ---- Telemetry ----
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Launcher velocity", "%.0f", launcher.getVelocity());
        telemetry.update();
    }

    /** Applies deadzone, then cubes the input (cubing keeps the sign). */
    private double shape(double v) {
        if (Math.abs(v) < DEADZONE) {
            return 0;
        }
        return v * v * v;
    }

    void arcadeDrive(double forward, double rotate) {
        leftPower = forward + rotate;
        rightPower = forward - rotate;

        // Keep powers within [-1, 1] without changing the turn ratio
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    void brakeMode() {
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);
    }

    void floatMode() {
        leftDrive.setZeroPowerBehavior(FLOAT);
        rightDrive.setZeroPowerBehavior(FLOAT);
    }

    void launch() {
        if (gamepad1.right_bumper) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            launcher.setVelocity(0);
        }

        if (gamepad1.right_bumper && launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
            windmillServo.setPower(1);
            intakePower += 0.5;
        } else {
            windmillServo.setPower(0);
        }
    }
}
