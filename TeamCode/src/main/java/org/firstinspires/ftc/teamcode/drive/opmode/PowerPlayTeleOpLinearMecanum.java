package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.GamepadHelper;
import org.firstinspires.ftc.teamcode.drive.IntakeSlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(name="MecanumDrve Drive", group = "Linear Opmode")
public class PowerPlayTeleOpLinearMecanum extends LinearOpMode {




    @Override
    public void runOpMode() throws InterruptedException {

        // initialize all the subsystems: 1. drivetrain,  2 intake+slide
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        IntakeSlideSubsystem intakeSlide = new IntakeSlideSubsystem(hardwareMap);

        double leftStickMultiplierX, leftStickMultiplierY;
        GamepadHelper leftStickX = new GamepadHelper();
        leftStickX.init();
        GamepadHelper leftStickY = new GamepadHelper();
        leftStickY.init();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();



        while (!isStopRequested()) {

            // drivebase control loop
            leftStickMultiplierX = leftStickX.getGamepadStickRampingMultiplier(gamepad1.left_stick_x);
            leftStickMultiplierY = leftStickY.getGamepadStickRampingMultiplier(gamepad1.left_stick_y);
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * leftStickMultiplierY ,
                            -gamepad1.left_stick_x * leftStickMultiplierX ,
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("X", poseEstimate.getX());
            telemetry.addData("Y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("leftStick x Multiplier", leftStickMultiplierX);
            telemetry.addData("leftStick y Multiplier", leftStickMultiplierY);
            telemetry.addData("GamePad leftStick x Input", gamepad1.left_stick_x);
            telemetry.addData("GamePad leftStick y Input", gamepad1.left_stick_y);

            // intake control loop
            intakeSlide.run(gamepad1, gamepad2);
            telemetry.addData("Current Slide Position", intakeSlide.getCurrentSlidePosition());
            telemetry.addData(intakeSlide.getCurrentCaption(), intakeSlide.getCurrentStatus());

            // publish all the telemetry at once
            telemetry.update();
        }

    }





}
