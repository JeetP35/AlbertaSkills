package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class BigRockBackup extends LinearOpMode {

    private DcMotor flMotor, frMotor, blMotor, brMotor;
    private IMU imu;

//    private CRServo wristLeft, wristRight;
//    private Servo claw;

    private DcMotor armMotor;
    private DcMotor armRotation;

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

        Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);

//        wristLeft = hardwareMap.get(CRServo.class, "wristLeft");
//        wristRight = hardwareMap.get(CRServo.class, "wristRight");
//
//        claw = hardwareMap.get(Servo.class, "claw");

        armMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        armRotation = hardwareMap.get(DcMotor.class, "ArmRotation");

        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //ExtendPosition
        armRotation.setTargetPosition(0);
        armRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        double[] powers;

        waitForStart();

        while (opModeIsActive()) {
            String driveMode = "RobotCentric";
            powers = robotCentricDrive(); // default

            double wristRightPower = gamepad1.right_trigger;
            double wristLeftPower = gamepad1.left_trigger;

            double rotations=312;
            double gear=5;
            double countsRev=rotations*gear;
            double degree=countsRev/360;

            boolean armUp = gamepad1.dpad_up;
            boolean armDown = gamepad1.dpad_down;
            int armPosition = armMotor.getCurrentPosition();
            double armMaxLimit = 0;
            double armMinLimit = 0;

            boolean armRight = gamepad1.dpad_right;
            boolean armLeft = gamepad1.dpad_left;
            int armRotationPosition = armRotation.getCurrentPosition();

            flMotor.setPower(-powers[0]);
            frMotor.setPower(-powers[1]);
            blMotor.setPower(powers[2]);
            brMotor.setPower(powers[3]);

//            wristRight.setPower(wristRightPower);
//            wristLeft.setPower(wristLeftPower);

            if (armUp) {
                armMotor.setPower(0.40);
                armMotor.setTargetPosition(armPosition + (int) (degree * 90));
            }
            else if (armDown) {
                armMotor.setPower(0.40);
                armMotor.setTargetPosition(armPosition - (int) (degree * 90));
            }
            else {
                armMotor.setPower(1.00);
                armMotor.setTargetPosition(armPosition);
            }

            if (armRight) {
                armRotation.setPower(0.40);
                armRotation.setTargetPosition(armRotationPosition + (int) (degree * 90));
            }
            else if (armLeft) {
                armRotation.setPower(0.40);
                armRotation.setTargetPosition(armRotationPosition - (int) (degree * 90));
            }
            else {
                armRotation.setPower(1.00);
                armRotation.setTargetPosition(armRotationPosition);
            }

//            if (gamepad1.a) {
//                claw.setPosition(1.00);
//            } else if (gamepad1.b) {
//                claw.setPosition(-1.00);
//            }

            if (gamepadRateLimit.hasExpired() && gamepad1.a) {
                imu.resetYaw();
                gamepadRateLimit.reset();
            } else if (gamepad1.a) {
                imu.resetYaw();
            }

            telemetry.addData("Active Drive Mode: ", driveMode);
//            telemetry.addData("WristR Power: ", wristRight.getPower());
//            telemetry.addData("WristL Power: ", wristLeft.getPower());
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

        return new double[] {
                Range.clip((ForwardBack + LeftRight + Rotate) * SpeedMultiplier, -1.0, 1.0), //Fl
                Range.clip((ForwardBack - LeftRight - Rotate) * SpeedMultiplier, -1.0, 1.0), //FR
                Range.clip((ForwardBack - LeftRight + Rotate) * SpeedMultiplier, -1.0, 1.0), //BL
                Range.clip((ForwardBack + LeftRight - Rotate) * SpeedMultiplier, -1.0, 1.0)  //BR
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

        return new double[] {
                leftPower, rightPower, leftPower, rightPower
        };
    }
}