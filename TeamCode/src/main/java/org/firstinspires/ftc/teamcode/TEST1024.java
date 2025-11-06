package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "DRIVE11/5")
public class TEST1024 extends LinearOpMode {

    public Motor leftFront, leftBack, rightFront, rightBack;
    public DriveSubsystem drive;
    public GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() throws InterruptedException {
        // Motor initialization
        leftFront = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312);
        rightFront = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312);
        leftBack = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_312);
        rightBack = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_312);

        leftFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setRunMode(Motor.RunMode.RawPower);
        rightFront.setRunMode(Motor.RunMode.RawPower);
        leftBack.setRunMode(Motor.RunMode.RawPower);
        rightBack.setRunMode(Motor.RunMode.RawPower);

        leftFront.setInverted(true);
        leftBack.setInverted(true);
        rightFront.setInverted(true);
        rightBack.setInverted(true);

        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Pinpoint odometry
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");
        drive = new DriveSubsystem(leftFront, rightFront, leftBack, rightBack);

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(5.5, 5, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();

        waitForStart();

        if (opModeIsActive()) {
            turnTo(180);
            drive.driveRobotCentricPowers(0,0,0);
            telemetry.addLine("DONE");
            telemetry.update();
        }
    };

    /**
     * Drive to a field coordinate (x, y) in inches.
     * Converts field X/Y PID outputs to robot-centric strafe/forward velocities.
     */
    public void driveTo(double targetX, double targetY) {
        PIDController pidX = new PIDController(0.03, 0.001, 0.01);
        PIDController pidY = new PIDController(0.03, 0.001, 0.01);

        pidX.setTolerance(0.5);
        pidY.setTolerance(0.5);

        while (opModeIsActive()) {
            pinpoint.update();

            double currentX = pinpoint.getPosX(DistanceUnit.INCH);
            double currentY = pinpoint.getPosY(DistanceUnit.INCH);

            double errorX = targetX - currentX;
            double errorY = targetY - currentY;

            // Stop condition
            if (Math.abs(errorX) < 0.5 && Math.abs(errorY) < 0.5) break;

            // PID outputs in field coordinates
            double fieldPowerX = pidX.calculate(currentX, targetX);
            double fieldPowerY = pidY.calculate(currentY, targetY);

            // Clamp powers
            fieldPowerX = Math.max(-0.9, Math.min(0.9, fieldPowerX));
            fieldPowerY = Math.max(-0.9, Math.min(0.9, fieldPowerY));

            // Convert field X/Y to robot-centric strafe/forward
            double robotHeading = Math.toRadians(pinpoint.getHeading(AngleUnit.DEGREES));
            double robotStrafe = fieldPowerX * Math.cos(robotHeading) + fieldPowerY * Math.sin(robotHeading);
            double robotForward = -fieldPowerX * Math.sin(robotHeading) + fieldPowerY * Math.cos(robotHeading);

            // Drive
            drive.driveRobotCentricPowers(robotStrafe, robotForward, 0);

            // Telemetry
            telemetry.addData("CurrentX", currentX);
            telemetry.addData("CurrentY", currentY);
            telemetry.addData("ErrorX", errorX);
            telemetry.addData("ErrorY", errorY);
            telemetry.addData("Strafe", robotStrafe);
            telemetry.addData("Forward", robotForward);
            telemetry.update();
        }
        // Stop motors when target reached
        drive.driveRobotCentricPowers(0, 0, 0);
    }

    /**
     * Turn to a heading in degrees (field coordinates)
     */
    public void turnTo(double targetHeading) {
        PIDController pid = new PIDController(0.023, 0.098, 0.004);
        pid.setTolerance(0.01);
        pid.setSetPoint(0.02);  // target error = 0

        while (opModeIsActive()) {
            pinpoint.update();

            double currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);
            double error = angleWrap(targetHeading - currentHeading);
            if (error <= 0.08) break;
            double powerTurn = pid.calculate(error, 0);
            powerTurn = Math.max(-0.9, Math.min(0.9, powerTurn));

            drive.driveRobotCentricPowers(0, 0, powerTurn);

            telemetry.addData("CurrentHeading", currentHeading);
            telemetry.addData("Error", error);
            telemetry.addData("PowerTurn", powerTurn);
            telemetry.update();
        }
        drive.driveRobotCentricPowers(0, 0, 0);
    }

    private double angleWrap(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
}
