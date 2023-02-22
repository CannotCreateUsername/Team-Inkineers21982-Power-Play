package org.firstinspires.ftc.teamcode.drive.opmode.pp.autos;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Cone;
import org.firstinspires.ftc.teamcode.drive.IntakeSlideSubsystemAuto;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class LeftTEST {
    enum DriveState {
        TRAJECTORY1,
        DROP_OFF,
        PARK,
        IDLE,
    }
    DriveState driveState = DriveState.IDLE;
    private ElapsedTime runtime = new ElapsedTime();

    public void followPath(SampleMecanumDrive drive, IntakeSlideSubsystemAuto intakeSlide, Cone cone, int parkDistance, LinearOpMode op) {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .forward(50)
                .strafeRight(8)
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(trajSeq1.end())
                .strafeRight(16)
                .strafeLeft(parkDistance)
                .build();

        if (op.isStopRequested()) return;
        runtime.reset();
        drive.followTrajectorySequence(trajSeq1);
        cone.align(IntakeSlideSubsystemAuto.LiftState.MEDIUM, false);
        drive.followTrajectorySequence(park);
        intakeSlide.liftState = IntakeSlideSubsystemAuto.LiftState.REST;
        intakeSlide.run();
    }

    public void followPath2(SampleMecanumDrive drive, IntakeSlideSubsystemAuto intakeSlide, Cone cone, int parkDistance, LinearOpMode op) {
        Pose2d startPose = new Pose2d(-62,-10,Math.toRadians(0));
        Pose2d dropOff = new Pose2d(-25, -10, Math.toRadians(90));

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(dropOff)
                .build();

        if (op.isStopRequested()) return;
        runtime.reset();
        drive.followTrajectorySequence(trajSeq1);
        intakeSlide.liftState = IntakeSlideSubsystemAuto.LiftState.REST;
        intakeSlide.run();
    }
}