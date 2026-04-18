package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Backup extends LinearOpMode {

    private DcMotor FlMotor;
    private DcMotor FrMotor;
    private DcMotor BlMotor;
    private DcMotor BrMotor;
    
    @Override
    public void runOpMode() {
        
        FlMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        FrMotor = hardwareMap.get(DcMotor.class, "FRMotor");
        BlMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        BrMotor = hardwareMap.get(DcMotor.class, "BRMotor");

        waitForStart();
        
        while (opModeIsActive()) {
            //Main Code
            double ForwardBack = -gamepad1.left_stick_y;
            double LeftRight = gamepad1.left_stick_x;
            double Rotate = -gamepad1.right_stick_x;

            double SpeedMultiplier = 1.00;

            if (gamepad1.left_bumper) {
                SpeedMultiplier = 0.5;
            } else if (gamepad1.right_bumper) {
                SpeedMultiplier = 0.25;
            }

            double[] powers = drive(ForwardBack, LeftRight, Rotate, SpeedMultiplier);

            FlMotor.setPower(powers[0]);
            FrMotor.setPower(powers[1]);
            BlMotor.setPower(powers[2]);
            BrMotor.setPower(powers[3]);
        }
    }

    public double[] drive(double ForwardBack, double LeftRight, double Rotate, double SpeedMultiplier) {
        double FLP = Range.clip((ForwardBack + LeftRight + Rotate) * SpeedMultiplier, -1.0, 1.0);
        double FRP = Range.clip(-(ForwardBack - LeftRight - Rotate) * SpeedMultiplier, -1.0, 1.0);
        double RLP = Range.clip((ForwardBack - LeftRight + Rotate) * SpeedMultiplier, -1.0, 1.0);
        double RRP = Range.clip(-(ForwardBack + LeftRight - Rotate) * SpeedMultiplier, -1.0, 1.0);

        return new double[]{FLP, FRP, RLP, RRP};
    }
}