package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "fullauto11/8")
public class FullAutoProgram extends LinearOpMode {

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

        leftFront.setInverted(false);
        leftBack.setInverted(true);
        rightFront.setInverted(true);
        rightBack.setInverted(false);

        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Pinpoint odometry
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");
        drive = new DriveSubsystem(leftFront, rightFront, leftBack, rightBack);

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-5.5, -5, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();

        waitForStart();

        if (opModeIsActive()) {
            driveTo(15,10);
            drive.driveRobotCentricPowers(0, 0, 0);
            idle();
            sleep(100);
            turnTo(180);
            drive.driveRobotCentricPowers(0, 0, 0);
            idle();
            sleep(100);
            driveTo(0,0);
            drive.driveRobotCentricPowers(0, 0, 0);
            idle();
            sleep(100);
        }
    }

    /**
     * Drive to a field coordinate (x, y) in inches.
     * Converts field X/Y PID outputs to robot-centric strafe/forward velocities.
     */
    public void driveTo(double targetX, double targetY) {
        leftFront.setInverted(false);
        leftBack.setInverted(true);
        rightFront.setInverted(true);
        rightBack.setInverted(false);
        PIDController pidX = new PIDController(0.07, 0.006, 0.006);
        PIDController pidY = new PIDController(0.04, 0.038, 0.003);

        pidX.setTolerance(0.05);
        pidY.setTolerance(0.5);
        pidX.setSetPoint(targetX);
        pidY.setSetPoint(targetY);
        while (opModeIsActive() && (!pidY.atSetPoint() || !pidX.atSetPoint())) {
            pinpoint.update();

            double currentX = pinpoint.getPosX(DistanceUnit.INCH);
            double currentY = pinpoint.getPosY(DistanceUnit.INCH);

            double errorX = targetX - currentX;
            double errorY = targetY - currentY;

            // Stop condition
            if (Math.abs(errorX) < 0.3 && Math.abs(errorY) < 0.3) break;

            // PID outputs in field coordinates
            double fieldPowerX = pidX.calculate(currentX, targetX);
            double fieldPowerY = pidY.calculate(currentY, targetY);

            // Clamp powers
            fieldPowerX = Math.max(-0.4, Math.min(0.4, fieldPowerX));
            fieldPowerY = Math.max(-0.4, Math.min(0.4, fieldPowerY));

            // Convert field X/Y to robot-centric strafe/forward using transformation matrix
            double robotHeading = Math.toRadians(pinpoint.getHeading(AngleUnit.DEGREES));
            double robotStrafe = fieldPowerX * Math.cos(robotHeading) + fieldPowerY * Math.sin(robotHeading);
            double robotForward = -fieldPowerX * Math.sin(robotHeading) + fieldPowerY * Math.cos(robotHeading);

            // Drive
            drive.driveRobotCentricPowers(robotStrafe, robotForward,0);

            // Telemetry
            telemetry.addData("CurrentX", currentX);
            telemetry.addData("CurrentY", currentY);
            telemetry.addData("ErrorX", errorX);
            telemetry.addData("ErrorY", errorY);
            telemetry.addData("Strafe", fieldPowerX);
            telemetry.addData("Forward", fieldPowerY);
            telemetry.update();
        }

    }

    /**
     * Turn to a heading in degrees (field coordinates)
     */
    public void turnTo(double targetHeading) {
        leftFront.setInverted(true);
        leftBack.setInverted(true);
        rightFront.setInverted(true);
        rightBack.setInverted(true);
        PIDController pid = new PIDController(0.02, 0.098, 0.003);
        pid.setTolerance(0.01);
        pid.setSetPoint(0);  // target error = 0

        while (opModeIsActive()) {
            pinpoint.update();

            double currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);
            double error = angleWrap(targetHeading - currentHeading);
            if (Math.abs(error) <= 0.08) {
                break;
            }
            double powerTurn = pid.calculate(error, 0);
            powerTurn = Math.max(-0.9, Math.min(0.9, powerTurn));

            drive.driveRobotCentricPowers(0, 0, powerTurn);

            telemetry.addData("CurrentHeading", currentHeading);
            telemetry.addData("Error", error);
            telemetry.addData("PowerTurn", powerTurn);
            telemetry.update();
        }
        drive.driveRobotCentricPowers(0, 0, 0);
        leftFront.setInverted(false);
        leftBack.setInverted(true);
        rightFront.setInverted(true);
        rightBack.setInverted(false);

    }

    private double angleWrap(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
}
