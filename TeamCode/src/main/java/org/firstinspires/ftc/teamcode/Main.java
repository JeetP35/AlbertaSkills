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
public class Main extends LinearOpMode {

    private Drive Drive;

    private DcMotor flMotor;
    private DcMotor frMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;

    private IMU imu;

    public String driveMode;
    public double[] powers;

    @Override
    public void runOpMode() {
        flMotor = hardwareMap.get(DcMotor.class, "FrontLeft");
        frMotor = hardwareMap.get(DcMotor.class, "FrontRight");
        blMotor = hardwareMap.get(DcMotor.class, "BackLeft");
        brMotor = hardwareMap.get(DcMotor.class, "BackRight");

        flMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.REVERSE);
        frMotor.setDirection(DcMotor.Direction.FORWARD);
        brMotor.setDirection(DcMotor.Direction.FORWARD);

        Drive = new Drive(flMotor, frMotor, blMotor, brMotor);

        Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);

        waitForStart();
        while (opModeIsActive()) {
            if (driveMode.equals("robotCentric")) {
                double ForwardBack = -gamepad1.left_stick_y;
                double LeftRight = gamepad1.left_stick_x;
                double Rotate = -gamepad1.right_stick_x;

                powers = Drive.robotCentric(ForwardBack, LeftRight, Rotate);
            } else if (driveMode.equals("fieldOriented")) {
                double leftY = -gamepad1.left_stick_y;
                double leftX = gamepad1.left_stick_x;
                double rightX = gamepad1.right_stick_x;

                powers = Drive.fieldOriented(leftY, leftX, rightX);
            } else if (driveMode.equals("tankDrive")) {
                powers = Drive.tankDrive();
            }

            if (gamepadRateLimit.hasExpired() && gamepad1.a) {
                imu.resetYaw();
                gamepadRateLimit.reset();
            } else if (gamepad1.a) {
                imu.resetYaw();
            }

            flMotor.setPower(powers[0]);
            frMotor.setPower(powers[1]);
            blMotor.setPower(powers[2]);
            brMotor.setPower(powers[3]);

            telemetry.addData("Active Drive Mode: ", driveMode);
            telemetry.update();
        }
    }
}
