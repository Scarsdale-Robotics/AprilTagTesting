package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "strafe11/7")
public class TEST1024 extends LinearOpMode {
    public WebcamName webcamName;
    public VisionPortal vp;
    public AprilTagProcessor aprilTagProcessor;
    private final Size CAMERA_RESOLUTION = new Size(640, 480);
    public AprilTagDetection lastTagDetected;

    @Override
    public void runOpMode() throws InterruptedException {
        this.webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        this.aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .build();
        this.vp = new VisionPortal.Builder()
                .setCamera(webcamName)
                .enableLiveView(true)
                .setCameraResolution(CAMERA_RESOLUTION)
                .addProcessor(aprilTagProcessor)
                .setAutoStartStreamOnBuild(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setLiveViewContainerId(hardwareMap.appContext.getResources().getIdentifier(
                                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()))
                .build();
        vp.setProcessorEnabled(aprilTagProcessor, true);
        waitForStart();
        while (opModeIsActive()) {
            detectTags();
            if (lastTagDetected != null) {
                telemetry.addData("Proximity", xInchRadius(15, lastTagDetected));
            }
            if (getTargetTag(21,22,23) != null) {
                int motifId = getTargetTag(21,22,23).id;
                if (motifId == 21) telemetry.addLine("GPP");
                else if (motifId == 22) telemetry.addLine("PGP");
                else telemetry.addLine("PPG");
            }
            telemetry.update();
        }
    }
    public boolean xInchRadius(int radius, AprilTagDetection target) {
        return target.ftcPose.range <= radius;
    }
    public void detectTags() {
        ArrayList<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        if (detections != null && !detections.isEmpty()) {
            for (AprilTagDetection tag : detections) {
                telemetry.addLine("AprilTag Detected.");
                telemetry.addData("ID", tag.id);
                telemetry.addData("X (Sideways offset)", tag.ftcPose.x);
                telemetry.addData("Y (Forward/Back Offset)", tag.ftcPose.y);
                telemetry.addData("Z", tag.ftcPose.z);
                telemetry.addData("Bearing", tag.ftcPose.bearing);
                telemetry.addData("Yaw", tag.ftcPose.yaw);
                telemetry.addData("Range: ", tag.ftcPose.range);
                lastTagDetected = tag;
            }
        } else {
            lastTagDetected = null; // clear old tag when none detected
        }
    }
    public AprilTagDetection getTargetTag(int targetId1, int targetId2, int targetId3) {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        AprilTagDetection sol = null;
        for (AprilTagDetection tag : detections) {
            if (tag.id == targetId1 || tag.id == targetId2 || tag.id == targetId3) {
                sol = tag;
                break;
            }
        }
        return sol;
    }
}
