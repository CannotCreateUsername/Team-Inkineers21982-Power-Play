package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.IntakeSlide;
import org.firstinspires.ftc.teamcode.drive.IntakeSlideSubsystem2;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Auto RIGHT Strafe (A2 or F5)", group="Linear Opmode")
public class PowerPlayRightSideAutoWithOdometryStrafeOnly extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException  {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // intake
        IntakeSlideSubsystem2 intakeSlide2 = new IntakeSlideSubsystem2();
        intakeSlide2.init(hardwareMap);

         /*
            read: Trajectories Overview
            https://learnroadrunner.com/trajectories.html#trajectories-vs-paths

            benefit of sequencer
            https://learnroadrunner.com/trajectory-sequence.html#overview

            reminder:
            Keep this in mind as the turn function will go counter-clockwise.
            all angles are in radian
            each tile is 24 inches (2 feet) ref: https://learnroadrunner.com/trajectories.html#coordinate-system
            always call drive.setPoseEstimate(startPose) to orient the drive ;

         */

        // we assume A2/F5 is starting point, the robot back is facing the wall
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);


        /**
         * Pickup Preload Cone
         * Stafe Left 24 inches
         * Straight 48 inches
         * Turn 45 degree  clockwise ;
         * Raise Intake to High Junction
         * Move forward 3 inches
         * Release Cone
         * Back 3 inches
         * Turn 45 degree  clockwise ;
         */


        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL_MEDIUM, DriveConstants.MAX_ANG_ACCE_MEDIUM)
                .setConstraints(SampleMecanumDrive.VEL_CONSTRAINT ,SampleMecanumDrive.ACCEL_CONSTRAINT) // max speed
                .addTemporalMarker(() -> {
                    // intake code goes here:
                    intakeSlide2.setIntakePower(IntakeSlide.IntakeState.IN);
                })
                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    // intake code goes here:
                    intakeSlide2.setIntakePower(IntakeSlide.IntakeState.STOP);
                })
                .waitSeconds(.5)
                .forward(1)
                .strafeLeft(24)
                .forward(49)
                .strafeLeft(10)
                .addTemporalMarker(() -> {
                    intakeSlide2.runToPosition(intakeSlide2.targetPositionHigh);
                })
                .waitSeconds(3)
                .back(4.5)
                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    // intake code goes here:
                    intakeSlide2.setIntakePower(IntakeSlide.IntakeState.OUT);
                })
                .waitSeconds(3)
                .addTemporalMarker(() -> {
                    // intake code goes here:
                    intakeSlide2.setIntakePower(IntakeSlide.IntakeState.STOP);
                })
                .waitSeconds(.5)
                .forward(7)
                .addTemporalMarker(() -> {
                    // intake code goes here:
                    // moveSlide(intakeSlide2, intakeSlide2.targetPositionHigh, 4);
                    intakeSlide2.runToPosition(intakeSlide2.targetPositionRest);
                })
                .waitSeconds(3)
                .strafeRight(9.75)
                .back(48)
                .resetConstraints()
                //.back(46)
                .build();


        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(trajSeq);




        // the last thing auto should do is move slide back to rest
        moveSlide(intakeSlide2, intakeSlide2.targetPositionRest, 30);

    }


    /**
     *
     * @param slides
     * @param position
     * @param timeoutS
     */
    public void moveSlide(IntakeSlide slides, int position, double timeoutS){

        ElapsedTime runtime = new ElapsedTime();


        // NOTE all while loop in op mode should check for
        // opModeIsActive
        while ( opModeIsActive() &&
                (runtime.seconds() < timeoutS)) {

            // run and  keep the position until timeout
            slides.runToPosition(position);

            // Display it for the driver.
            telemetry.addData("Slide to",  " %7d", position);
            telemetry.update();
        }
    }


}