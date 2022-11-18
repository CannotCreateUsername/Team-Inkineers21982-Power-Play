package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.GamepadHelper;
import org.firstinspires.ftc.teamcode.drive.IntakeSlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.IntakeSlideSubsystem2;
import org.firstinspires.ftc.teamcode.drive.IntakeSlide;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(name="Odyssea Drive", group = "Linear Opmode")
public class PowerPlayTeleOpLinearMecanum extends LinearOpMode {




    @Override
    public void runOpMode() throws InterruptedException {

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        // initialize all the subsystems: 1. drivetrain,  2 intake+slide
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        IntakeSlideSubsystem intakeSlide = new IntakeSlideSubsystem();
        intakeSlide.init(hardwareMap);
        IntakeSlideSubsystem2 intakeSlide2 = new IntakeSlideSubsystem2();
        intakeSlide2.init(hardwareMap);

        // by default , use Drive Control #1
        IntakeSlide currentIntakeSlide = intakeSlide;

        double leftStickMultiplierX, leftStickMultiplierY, rightStickMultiplierX;
        GamepadHelper leftStickX = new GamepadHelper();
        leftStickX.init();
        GamepadHelper leftStickY = new GamepadHelper();
        leftStickY.init();
        GamepadHelper rightStickX = new GamepadHelper();
        rightStickX.init();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();



        while (opModeIsActive()) {

            // drivebase control loop
            leftStickMultiplierX = leftStickX.getGamepadStickRampingMultiplier(gamepad1.left_stick_x);
            leftStickMultiplierY = leftStickY.getGamepadStickRampingMultiplier(gamepad1.left_stick_y);
            rightStickMultiplierX = rightStickX.getGamepadStickRampingMultiplier(gamepad1.right_stick_x);
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * leftStickMultiplierY ,
                            -gamepad1.left_stick_x * leftStickMultiplierX ,
                            -gamepad1.right_stick_x * rightStickMultiplierX
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


            // switch between two subsystem
            if (gamepad1.back || gamepad2.back){
                currentIntakeSlide = intakeSlide;
            } else if (gamepad1.start || gamepad2.start){
                currentIntakeSlide = intakeSlide2;
            }
            // use the abstract class interface to call the run code. but the actual implementaiton
            // can vary between intakeSlide and intakeSlide2
            // so at any point in time, only one drive control logic is being used
            currentIntakeSlide.run(gamepadEx1, gamepadEx2);
            telemetry.addData("Current Slide Position 1", currentIntakeSlide.getCurrentSlidePosition());
            telemetry.addData("Current State 1", currentIntakeSlide.getCurrentState());
            telemetry.addData("How many DpadUp?", intakeSlide.getDpadPressed());
            telemetry.addData(intakeSlide.getCurrentCaption(), currentIntakeSlide.getCurrentStatus());
            telemetry.addData("Current Control", currentIntakeSlide);

            // publish all the telemetry at once
            telemetry.update();
        }

    }





}
