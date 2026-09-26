package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import java.util.List;
//code finds side bot is on and tracks apriltags
@TeleOp(name = "FTC AprilTag Lookup Tracking", group = "Tracking")
public class AprilTagLookupTracking extends LinearOpMode {

    // Simple container class to mimic the Python tuple return type from f(c)
    static class TagData {
        int number;
        String alliance;
        int direction;

        TagData(int number, String alliance, int direction) {
            this.number = number;
            this.alliance = alliance;
            this.direction = direction;
        }
    }

    // Java translation of your python f(c) classification logic
    public TagData classifyTag(int id) {
        if (id == 31 || id == 30) return new TagData(2, "red", 1);
        if (id == 32 || id == 33) return new TagData(2, "red", -1);
        if (id == 34 || id == 35) return new TagData(1, "red", 1);
        if (id == 36 || id == 37) return new TagData(1, "red", -1);
        if (id == 38 || id == 39) return new TagData(1, "blue", 1);
        if (id == 40 || id == 41) return new TagData(1, "blue", -1);
        if (id == 42 || id == 43) return new TagData(2, "blue", 1);
        if (id == 44 || id == 45) return new TagData(2, "blue", -1); 
      //positions where the bot needs to centre 1 and 2 shows what side we are on and colour we got and where to move in posiiton to the tag
        return null; // For tags outside your targeted scope array
    }

    @Override
    public void runOpMode() {
        // 1. Initialize the FTC AprilTag Processor
        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                // Match your custom camera parameters [fx, fy, cx, cy]
                .setLensIntrinsics(1430.0, 1430.0, 320.0, 240.0)
                // Convert 0.165 meters tag size into inches (0.165 * 39.37)
                .setTagSize(6.496) 
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();

        // 2. Build the camera frame portal matching your 640x480 criteria
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new org.opencv.core.Size(640, 480))
                .build();

        telemetry.addData("Status", "Initialized. System Standby.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    double t_z = detection.ftcPose.z;
                    // finds roll of the apriltag
                    // Replicate the exact rotation matrix Roll angle calculation
                    Mat R = detection.rawPose.R; 
                    double r10 = R.get(1, 0)[0]; // Extract numeric array values natively
                    double r00 = R.get(0, 0)[0]; 
                    
                    double rollRadians = Math.atan2(r10, r00);
                    double rollDegrees = Math.toDegrees(rollRadians);

                    // Execute lookup logic for metadata classifications
                    TagData mappedInfo = classifyTag(detection.id);

                    // Push metrics cleanly to the Driver Station display logs
                    telemetry.addLine(String.format("Raw Tag ID: %d", detection.id));
                    if (mappedInfo != null) {
                        telemetry.addLine(String.format("Classification -> Num: %d | Alliance: %s | Dir: %d", 
                                mappedInfo.number, mappedInfo.alliance, mappedInfo.direction));
                    } else {
                        telemetry.addLine("Classification -> Unknown Target Tag");
                    }
                    telemetry.addData("Distance", String.format("%.2fm", t_z));
                    telemetry.addData("Roll Offset", String.format("%.1f°", rollDegrees));
                    telemetry.addLine("--------------------------------");
                }
            }
            
            telemetry.update();
            sleep(20);
        }

        visionPortal.close();
    }
}
