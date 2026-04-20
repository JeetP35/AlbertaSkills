package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

public class Drive {
    private final DcMotor flMotor,frMotor,blMotor, brMotor;
    private IMU imu;


    public double ForwardBack, LeftRight, Rotate;

    public Drive(DcMotor flMotor, DcMotor frMotor, DcMotor blMotor, DcMotor brMotor) {
        this.flMotor = flMotor;
        this.frMotor = frMotor;
        this.blMotor = blMotor;
        this.brMotor = brMotor;
    }

    public double[] robotCentric(double ForwardBack, double LeftRight, double Rotate) {
        double SpeedMultiplier = 1.00;
        if (gamepad1.left_bumper) {
            SpeedMultiplier = 0.5;
        } else if (gamepad1.right_bumper) {
            SpeedMultiplier = 0.25;
        }

        // Calculate power for each motor and ensure it stays within -1.0 to 1.0 and return them as a list
        return new double[] {
                Range.clip((ForwardBack + LeftRight + Rotate) * SpeedMultiplier, -1.0, 1.0),
                Range.clip((ForwardBack - LeftRight - Rotate) * SpeedMultiplier, -1.0 , 1.0),
                Range.clip((ForwardBack - LeftRight + Rotate) * SpeedMultiplier, -1.0, 1.0),
                Range.clip((ForwardBack + LeftRight - Rotate) * SpeedMultiplier, -1.0, 1.0)
        };
    }

    public double[] fieldOriented(double leftY, double leftX, double rightX) {
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

    public double[] tankDrive () {
        double rightPower = -gamepad1.right_stick_y;
        double leftPower = -gamepad1.left_stick_y;

        return new double[] {
                leftPower, rightPower, leftPower, rightPower
        };
    }
}
