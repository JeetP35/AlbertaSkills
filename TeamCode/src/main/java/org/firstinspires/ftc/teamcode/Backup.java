package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.subsystems.*;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class Backup extends LinearOpMode {

    private DcMotor flMotor, frMotor, blMotor, brMotor;
    private IMU imu;

    private String driveMode = "RobotCentric";

    @Override
    public void runOpMode() {

        flMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        frMotor = hardwareMap.get(DcMotor.class, "FRMotor");
        blMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        brMotor = hardwareMap.get(DcMotor.class, "BRMotor");

        flMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.REVERSE);
        frMotor.setDirection(DcMotor.Direction.FORWARD);
        brMotor.setDirection(DcMotor.Direction.FORWARD);

        Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);

        double[] powers;

        waitForStart();
        
        while (opModeIsActive()) {
            if (driveMode.equals("FieldOriented")) {
                powers = fieldOriented(); //Field Oriented
            } else if (driveMode.equals("TankDrive")) {
                powers = tankDrive(); //Tank Drive
            } else {
                powers = robotCentricDrive(); // default
            }

            flMotor.setPower(powers[0]);
            frMotor.setPower(powers[1]);
            blMotor.setPower(powers[2]);
            brMotor.setPower(powers[3]);

            if (gamepadRateLimit.hasExpired() && gamepad1.a) {
                imu.resetYaw();
                gamepadRateLimit.reset();
            } else if (gamepad1.a) {
                imu.resetYaw();
            }

            telemetry.addData("Active Drive Mode: ", driveMode);
            telemetry.update();
        }
    }

    public double[] robotCentricDrive() {
        double ForwardBack = -gamepad1.left_stick_y;
        double LeftRight = gamepad1.left_stick_x;
        double Rotate = -gamepad1.right_stick_x;

        double SpeedMultiplier = 1.00;

        if (gamepad1.left_bumper) {
            SpeedMultiplier = 0.5;
        } else if (gamepad1.right_bumper) {
            SpeedMultiplier = 0.25;
        }

        double max = Math.max(Math.abs(ForwardBack) + Math.abs(LeftRight) + Math.abs(Rotate), 1.0);

        return new double[]{
                Range.clip((ForwardBack + LeftRight + Rotate) / max, -1.0, 1.0) * SpeedMultiplier,
                Range.clip((ForwardBack - LeftRight - Rotate) / max, -1.0, 1.0) * SpeedMultiplier,
                Range.clip((ForwardBack - LeftRight + Rotate) / max, -1.0, 1.0) * SpeedMultiplier,
                Range.clip((ForwardBack + LeftRight - Rotate) / max, -1.0, 1.0) * SpeedMultiplier
        };
    }

    public double[] fieldOriented() {
        double leftY = -gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;

        double max = Math.max(Math.abs(leftX) + Math.abs(leftY) + Math.abs(rightX), 1);

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double adjustedLeftX = -leftY * Math.sin(heading) + leftX * Math.cos(heading);
        double adjustedLeftY = leftY * Math.cos(heading) + leftX * Math.sin(heading);

        double drivePower = 0.8 - (0.6 * gamepad1.right_trigger);

        return new double[] {
                ((adjustedLeftY + adjustedLeftX + rightX) / max) * drivePower, //Front Left
                ((adjustedLeftY - adjustedLeftX - rightX) / max) * drivePower, //Front Right
                ((adjustedLeftY - adjustedLeftX + rightX) / max) * drivePower, //Back Left
                ((adjustedLeftY + adjustedLeftX - rightX) / max) * drivePower  //Back Right
        };
    }

    public double[] tankDrive() {
        double rightPower = -gamepad1.right_stick_y;
        double leftPower = -gamepad1.left_stick_y;

        return new double[]{
                leftPower, rightPower, leftPower, rightPower
        };
    }
}