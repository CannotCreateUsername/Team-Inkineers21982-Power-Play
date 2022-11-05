package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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


    public enum GamePadState {
        POSITIVE,
        NETURAL,
        NEGATIVE
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize all the subsystems: 1. drivetrain,  2 intake+slide
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        IntakeSlideSubsystem intakeSlide = new IntakeSlideSubsystem(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();



        /*
        Time Base Ramping
        If the driver keep on pressing the gamepad, the sensitivity of the gamepad input will increase over time
        */
        ElapsedTime xGamePadTimer = new ElapsedTime();
        boolean xRamp = false;
        GamePadState prevoiusXGamePadState = GamePadState.NETURAL;
        GamePadState currentXGamePadState = GamePadState.NETURAL;
        // double  previousXGamePad = 0;

        double minXMulplier = 0.1;
        double maxXMultiplier = 0.75;
        double incrementMultipler = 0.1;
        double xGamePadMultipler = 0;
        double yGamePadMultipler = 0;

        /**
         double maxMultiplier = 0.75;
         double minMultiplier = 0.25;
         double threshHold1 = 0.75;

         double xGamePadMultipler = 0;
         double yGampePadMultipler = 0;
         */

        while (!isStopRequested()) {
            
            /*
            if(xRamp){
                xGamePadTimer.reset();
            }
            if (xGamePadTimer.milliseconds() > 10 ){
                
            }
            */

            /**
             *  OLD LOGIC:
             *  Ramp is true  when 
             *  1. gamePad.x change from 0 to any positive
             *  2. gamePad.x change from 0 any negative 
             *  3. gamePad.x change from any negative to any positive
             *  4. gamePad.x change from any positive to any negative 
             *  
             *  assumption:  3 and 4 are covered by 1 and 2 
             */
            /**
            if (previousXGamePad == 0 && gamepad1.left_stick_x != 0 ){
                xRamp = true;
                xGamePadMultipler = minXMulplier;
                xGamePadTimer.reset();
            }
             previousXGamePad = gamepad1.left_stick_x;
             */

            /**
            Ivan: this previous logic assume that we can capture the value of gamePad = 0.
             But since the moving of gamepad joystick is fast, we might not be able to store the value of 0 reliability

             so here is the new logic:
             Ramp is true  when there is any chagne of the game pad state
             i.e. from NEUTRAL TO NEGATIVE , NEUTRAL TO POSISTIVE, POSTIVE TO NEGATIVE, NEGATIVE TO POSITIVE ....
             */

            prevoiusXGamePadState = currentXGamePadState;
            if (gamepad1.left_stick_x < 0){
                currentXGamePadState = GamePadState.NETURAL;
            } else  if (gamepad1.left_stick_x > 0){
                currentXGamePadState = GamePadState.POSITIVE;
            } else {
                currentXGamePadState = GamePadState.NETURAL;
            }
            if (prevoiusXGamePadState != currentXGamePadState ){
                xRamp = true;
                xGamePadMultipler = minXMulplier;
                xGamePadTimer.reset();
            }
            
            if (xRamp && xGamePadTimer.milliseconds() > 10) {
                if (xGamePadMultipler <= maxXMultiplier) {
                    xGamePadMultipler += incrementMultipler;
                } else {
                    xRamp = false;
                }

                if (xGamePadMultipler > maxXMultiplier) {
                    xGamePadMultipler = maxXMultiplier;
                }

                xGamePadTimer.reset();
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y*0.75,
                            -gamepad1.left_stick_x*xGamePadMultipler,
                            -gamepad1.right_stick_x
                    )
            );

            /**
             * Two (X)  step ramp
             *
             * If gamepad.x is less than 0.5 , divide the power by a fixed amount (ReductionFactor)
             * Else , go the max power
             */
            /*
            if (  Math.abs(gamepad1.left_stick_x) < threshHold1  ){
                xGamePadMultipler = minMultiplier;
            } else {
                xGamePadMultipler = maxMultiplier;
            }

            if (  Math.abs(gamepad1.left_stick_y) < threshHold1  ){
                yGampePadMultipler = minMultiplier;
            } else {
                yGampePadMultipler = maxMultiplier;
            }

             */

            /*
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y*yGampePadMultipler,// -gamepad1.left_stick_y*0.75,
                            -gamepad1.left_stick_x*xGamePadMultipler,// -gamepad1.left_stick_x*0.75,
                            -gamepad1.right_stick_x
                    )
            );

             */

            /**

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y* -gamepad1.left_stick_y*0.75,
                            -gamepad1.left_stick_x* -gamepad1.left_stick_x*0.75,
                            -gamepad1.right_stick_x
                    )
            );
             */
            drive.update();

            intakeSlide.run(gamepad1, gamepad2);
            telemetry.addData(intakeSlide.getCurrentCaption(), intakeSlide.getCurrentStatus());

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("xMultiplier", xGamePadMultipler);
            // telemetry.addData("yMultiplier", yGampePadMultipler);
            telemetry.addData("x GamePad Input", gamepad1.left_stick_x);
            telemetry.addData("y GamePad Input", gamepad1.left_stick_y);

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }

    }





}
