package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp (name = "FieldCentricDrive")
public class FieldCentricTest extends LinearOpMode {
    public double speed = 0.5;
    public GoBildaPinpointDriver pinpoint;
    public Motor leftBack;
    public Motor leftFront;
    public Motor rightBack;
    public Motor rightFront;
    public DriveSubsystem drive;
    @Override
    public void runOpMode() throws InterruptedException {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-5.5,-1.5, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
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
        rightFront.setInverted(true);
        leftBack.setInverted(false);
        rightBack.setInverted(false);

        leftFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.drive = new DriveSubsystem(leftFront, rightFront, leftBack, rightBack);
        waitForStart();
        while (opModeIsActive()) {
            pinpoint.update();
            telemetry.addData("H", normalizeAngle(pinpoint.getHeading(AngleUnit.DEGREES)));
            telemetry.addData("X", pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("Y", pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.update();
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;

            double theta = Math.atan2(forward, strafe);
            double driveSpeed = Math.hypot(strafe, forward) * DriveConstants.MAX_FORWARD_SPEED;
            double turnVel = gamepad1.right_stick_x * RotationConstants.MAX_ANGULAR_VELOCITY;

            drive.driveFieldCentric(
                    theta,
                    driveSpeed,
                    turnVel,
                    Math.toRadians(normalizeAngle(pinpoint.getHeading(AngleUnit.DEGREES)))
            );
        }
    }
    public double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle <= -180) angle += 360;
        if (angle > 180) angle -= 360;
        return angle;
    }
}
