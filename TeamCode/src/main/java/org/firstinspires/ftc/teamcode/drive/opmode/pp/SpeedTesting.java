package org.firstinspires.ftc.teamcode.drive.opmode.pp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.GamepadHelper;
import org.firstinspires.ftc.teamcode.drive.IntakeSlideSubsystem4;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(name="Speed Testing", group = "Linear Opmode")
public class SpeedTesting extends LinearOpMode {

    private enum TurnState {
        STRAIGHT,
        LEFT,
        RIGHT,
        BACKWARDS
    }

    TurnState turnState;

    private double LeftXInput;
    private double LeftYInput;
    private double RightXInput;

    private double MaxSpeedMultiplier = 0.8;

    private TouchSensor sensorTouch1;
    private TouchSensor sensorTouch2;

    @Override
    public void runOpMode() throws InterruptedException {
        turnState = TurnState.STRAIGHT;
        sensorTouch1 = hardwareMap.get(TouchSensor.class, "touch1");
        sensorTouch2 = hardwareMap.get(TouchSensor.class, "touch2");

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        // initialize all the subsystems: 1. drivetrain,  2 intake+slide
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        IntakeSlideSubsystem4 intakeSlide = new IntakeSlideSubsystem4();
        intakeSlide.init(hardwareMap);


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            Pose2d poseEstimate = drive.getPoseEstimate();

            // keeps controls the same if robot is rotated 90 degrees in any direction
            switch (turnState) {
                case STRAIGHT:
                    LeftXInput = gamepad1.left_stick_x * MaxSpeedMultiplier * intakeSlide.dropOffMultiplier;
                    LeftYInput = gamepad1.left_stick_y * MaxSpeedMultiplier * intakeSlide.dropOffMultiplier;
                    RightXInput = gamepad1.right_stick_x * MaxSpeedMultiplier * intakeSlide.dropOffMultiplier;
                    if (gamepadEx1.wasJustReleased(GamepadKeys.Button.DPAD_LEFT)) {
                        drive.turn(Math.toRadians(90));
                        turnState = TurnState.LEFT;
                    } else if (gamepadEx1.wasJustReleased(GamepadKeys.Button.DPAD_RIGHT)) {
                        drive.turn(Math.toRadians(-90));
                        turnState = TurnState.RIGHT;
                    }
                    break;
                case LEFT:
                    LeftXInput = -gamepad1.left_stick_y * MaxSpeedMultiplier * intakeSlide.dropOffMultiplier;
                    LeftYInput = gamepad1.left_stick_x * MaxSpeedMultiplier * intakeSlide.dropOffMultiplier;
                    RightXInput = gamepad1.right_stick_x * MaxSpeedMultiplier * intakeSlide.dropOffMultiplier;
                    if (gamepadEx1.wasJustReleased(GamepadKeys.Button.DPAD_LEFT)) {
                        drive.turn(Math.toRadians(90));
                        turnState = TurnState.BACKWARDS;
                    } else if (gamepadEx1.wasJustReleased(GamepadKeys.Button.DPAD_RIGHT)) {
                        drive.turn(Math.toRadians(-90));
                        turnState = TurnState.STRAIGHT;
                    }
                    break;
                case RIGHT:
                    LeftXInput = gamepad1.left_stick_y * MaxSpeedMultiplier * intakeSlide.dropOffMultiplier;
                    LeftYInput = -gamepad1.left_stick_x * MaxSpeedMultiplier * intakeSlide.dropOffMultiplier;
                    RightXInput = gamepad1.right_stick_x * MaxSpeedMultiplier * intakeSlide.dropOffMultiplier;
                    if (gamepadEx1.wasJustReleased(GamepadKeys.Button.DPAD_LEFT)) {
                        drive.turn(Math.toRadians(90));
                        turnState = TurnState.STRAIGHT;
                    } else if (gamepadEx1.wasJustReleased(GamepadKeys.Button.DPAD_RIGHT)) {
                        drive.turn(Math.toRadians(-90));
                        turnState = TurnState.BACKWARDS;
                    }
                    break;
                case BACKWARDS:
                    LeftXInput = -gamepad1.left_stick_x * MaxSpeedMultiplier * intakeSlide.dropOffMultiplier;
                    LeftYInput = -gamepad1.left_stick_y * MaxSpeedMultiplier * intakeSlide.dropOffMultiplier;
                    RightXInput = gamepad1.right_stick_x * MaxSpeedMultiplier * intakeSlide.dropOffMultiplier;
                    if (gamepadEx1.wasJustReleased(GamepadKeys.Button.DPAD_LEFT)) {
                        drive.turn(Math.toRadians(90));
                        turnState = TurnState.RIGHT;
                    } else if (gamepadEx1.wasJustReleased(GamepadKeys.Button.DPAD_RIGHT)) {
                        drive.turn(Math.toRadians(-90));
                        turnState = TurnState.LEFT;
                    }
                    break;
            }
//
//            // Field centric view
//            Vector2d input = new Vector2d(
//                    -LeftYInput,
//                    -LeftXInput
//            ).rotated(-poseEstimate.getHeading());

//            LeftXInput = gamepad1.left_stick_x * leftStickMultiplierX * intakeSlide.dropOffMultiplier;
//            LeftYInput = gamepad1.left_stick_y * leftStickMultiplierY * intakeSlide.dropOffMultiplier;
//            RightXInput = gamepad1.right_stick_x * rightStickMultiplierX * intakeSlide.dropOffMultiplier;

            if (!sensorTouch1.isPressed() || !sensorTouch2.isPressed()) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -LeftYInput,
                                -LeftXInput,
                                -RightXInput
                        )
                );
                drive.update();
            } else if (sensorTouch1.isPressed() && sensorTouch1.isPressed()) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -0,
                                -0.1,
                                -0
                        )
                );
                drive.update();
            }



            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) {
                MaxSpeedMultiplier += 0.05;
            } else if (gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) {
                MaxSpeedMultiplier -= 0.05;
            }
            if (MaxSpeedMultiplier > 1) {
                MaxSpeedMultiplier = 1;
            } else if (MaxSpeedMultiplier < 0) {
                MaxSpeedMultiplier = 0;
            }

//            alignStick.run();

//            telemetry.addData("X", poseEstimate.getX());
//            telemetry.addData("Y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
//            telemetry.addData("leftStick x Multiplier", leftStickMultiplierX);
//            telemetry.addData("leftStick y Multiplier", leftStickMultiplierY);
//            telemetry.addData("GamePad leftStick x Input", gamepad1.left_stick_x);
//            telemetry.addData("GamePad leftStick y Input", gamepad1.left_stick_y);

            intakeSlide.run(gamepadEx1, gamepadEx2);
            intakeSlide.runIntake(gamepadEx1);
//            telemetry.addData("Current Slide Position 1", intakeSlide.getCurrentSlidePosition());
            telemetry.addData("Current State 1", intakeSlide.getCurrentState());
//            telemetry.addData("How many DpadUp?", intakeSlide.getDpadPressed());
            telemetry.addData("Current Control", intakeSlide);
//            telemetry.addData("Is intake pressed", intakeSlide3.getIntakePressed());
            telemetry.addData("Rotation", turnState.name());

            // Distance
//            telemetry.addData("range", String.format("%.01f cm", alignStick.getDistanceReadingCM()));
//            telemetry.addData("range", String.format("%.01f mm", alignStick.getDistanceReadingMM()));
//            telemetry.addData("Align State", alignStick.getAlignState());
//            telemetry.addData("Light State", alignStick.getLightState());
//            telemetry.addData("Align Multiplier", alignStick.getGameStickMultiplier());

            telemetry.addData("Servo Position", intakeSlide.getServoPosition());
            telemetry.addData("Intake State", intakeSlide.getIntakeState());
            telemetry.addData("Current Max Speed", MaxSpeedMultiplier);

            // publish all the telemetry at once
            telemetry.update();
        }
    }

}
