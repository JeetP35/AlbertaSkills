package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class Drive {
    private DcMotor flMotor;
    private DcMotor frMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;

    public double ForwardBack;
    public double LeftRight;
    public double Rotate;

    public Drive(DcMotor flMotor, DcMotor frMotor, DcMotor blMotor, DcMotor brMotor, double ForwardBack, double LeftRight, double Rotate) {
        this.flMotor = flMotor;
        this.frMotor = frMotor;
        this.blMotor = blMotor;
        this.brMotor = brMotor;

        this.ForwardBack = ForwardBack;
        this.LeftRight = LeftRight;
        this.Rotate = Rotate;
    }

    public void robotCentric() {
        double SpeedMultiplier = 1.00;
        if (gamepad1.left_bumper) {
            SpeedMultiplier = 0.5;
        } else if (gamepad1.right_bumper) {
            SpeedMultiplier = 0.25;
        }

        // Calculate power for each motor and ensure it stays within -1.0 to 1.0
        double FLP = Range.clip((ForwardBack + LeftRight + Rotate) * SpeedMultiplier, -1.0, 1.0);
        double FRP = Range.clip(-(ForwardBack - LeftRight - Rotate) * SpeedMultiplier, -1.0, 1.0);
        double RLP = Range.clip((ForwardBack - LeftRight + Rotate) * SpeedMultiplier, -1.0, 1.0);
        double RRP = Range.clip(-(ForwardBack + LeftRight - Rotate) * SpeedMultiplier,-1.0, 1.0);

        // Set motor power
        flMotor.setPower(FLP);
        frMotor.setPower(FRP);
        blMotor.setPower(RLP);
        brMotor.setPower(RRP);
    }

    public void fieldOriented() {
    }

    public void tankDrive () {
        double rightPower = -gamepad1.right_stick_y;
        double leftPower = -gamepad1.left_stick_y;

        flMotor.setPower(leftPower);
        flMotor.setPower(rightPower);
        blMotor.setPower(leftPower);
        brMotor.setPower(rightPower);
    }
}
