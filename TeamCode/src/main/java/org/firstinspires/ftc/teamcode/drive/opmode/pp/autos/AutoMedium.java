package org.firstinspires.ftc.teamcode.drive.opmode.pp.autos;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Cone;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.intakeslide.IntakeSlideSubsystemAuto;
import org.firstinspires.ftc.teamcode.drive.opmode.pp.AutoInterface;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class AutoMedium {
    enum DriveState {
        TRAJECTORY1,
        DROP_OFF,
        PARK,
        IDLE,
    }
    DriveState driveState = DriveState.IDLE;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean coneThere = false;

    LinearOpMode op;
    SampleMecanumDrive drive;
    IntakeSlideSubsystemAuto intakeSlide;
    Cone cone;
    AutoInterface positions = new AutoInterface();

    public void init(SampleMecanumDrive d, IntakeSlideSubsystemAuto i, Cone c, LinearOpMode o) {
        drive = d;
        intakeSlide = i;
        cone = c;
        op = o;
    }

    public void followPath(int parkDistance, int side) {
        // locations
        Pose2d pickUp = positions.ConeStack;
        Pose2d dropOff = positions.Medium;

        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        // forward/backwards does not need to be reversed
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .forward(2)
                .strafeLeft(24*side)
                .forward(46)
                .strafeRight(10*side)
                .build();
        TrajectorySequence drop = drive.trajectorySequenceBuilder(pickUp)
                .lineToLinearHeading(dropOff)
                .build();
        TrajectorySequence pickup = drive.trajectorySequenceBuilder(dropOff)
                .lineToLinearHeading(pickUp)
                .build();
        TrajectorySequence park = drive.trajectorySequenceBuilder(drop.end())
                .lineToLinearHeading(new Pose2d(parkDistance*side, -12, Math.toRadians(90)))
                .build();

        if (op.isStopRequested()) return;
        drive.followTrajectorySequence(trajSeq1);
        cone.drop = true;
        runtime.reset();
        while (runtime.seconds() < 1) {
            // wait.. add telemetry here
            op.telemetry.addData("Waiting", "to align");
            op.telemetry.update();
        }
        cone.align(IntakeSlideSubsystemAuto.LiftState.MEDIUM, false);
        drive.setPoseEstimate(dropOff);
        for (int i = 0; i < 1; i++) {
            drive.followTrajectorySequence(pickup);
            cone.pickUpCone();
            drive.followTrajectorySequence(drop);
            cone.align(IntakeSlideSubsystemAuto.LiftState.MEDIUM, coneThere);
            coneThere = true;
        }
        drive.followTrajectorySequence(park);
        intakeSlide.liftState = IntakeSlideSubsystemAuto.LiftState.REST;
        intakeSlide.run();
    }

//    public void followPath2() {
//        Pose2d pickUp = positions.LeftConeStack;
//        Pose2d dropOff = positions.Medium;
//
//        drive.setPoseEstimate(pickUp);
//
//        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(pickUp)
//                .lineToLinearHeading(dropOff)
//                .build();
//
//        if (op.isStopRequested()) return;
//        runtime.reset();
//        drive.followTrajectorySequence(trajSeq1);
//        intakeSlide.liftState = IntakeSlideSubsystemAuto.LiftState.REST;
//        intakeSlide.run();
//    }
}