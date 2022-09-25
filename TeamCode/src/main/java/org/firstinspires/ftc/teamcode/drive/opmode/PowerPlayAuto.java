package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="PowerPlay1", group="Linear Opmode")
public class PowerPlayAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException  {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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


        // We want to start A2 position
        Pose2d startPose = new Pose2d(-60, -34, 0);
        drive.setPoseEstimate(startPose);

        /*
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-45, -60), Math.toRadians(0))
                .splineTo(new Vector2d(-30, -20), Math.toRadians(90))
                .lineTo(new Vector2d(-30, -60))
                .build();

            new Pose2d(-60, -34, 0))
                .lineTo(new Vector2d(-60, -15))
                .splineTo(new Vector2d(-9, -9), Math.toRadians(45))
                .splineToConstantHeading(new Vector2d(-30, -60), Math.toRadians(45))
         */

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-60, -15))
                .splineTo(new Vector2d(-20, -9), Math.toRadians(0))
                .turn(Math.toRadians(45))
                .splineToConstantHeading(new Vector2d(-30, -60), Math.toRadians(45))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(trajSeq);
    }
}