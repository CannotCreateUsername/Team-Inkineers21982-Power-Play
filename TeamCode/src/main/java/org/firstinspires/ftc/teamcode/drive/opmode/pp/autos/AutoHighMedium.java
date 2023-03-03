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

public class AutoHighMedium {
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

    public Pose2d Start = positions.Start;
    public Pose2d Low = positions.Low;
    public Pose2d Medium = positions.Medium;
    public Pose2d High = positions.High;
    public Pose2d BottomHigh = positions.BottomHigh;

    public void init(SampleMecanumDrive d, IntakeSlideSubsystemAuto i, Cone c, LinearOpMode o, int startSide) {
        drive = d;
        intakeSlide = i;
        cone = c;
        op = o;

        // junctions
        Start = new Pose2d(34*startSide, -62, Math.toRadians(90));
        Low = new Pose2d(42*startSide, -12, Math.toRadians(90));
        Medium = new Pose2d(0*startSide, -12, Math.toRadians(90));
        High = new Pose2d(0*startSide, -12, Math.toRadians(-90));
        BottomHigh = new Pose2d(0, -12,  Math.toRadians(90));
    }

    public void followPath(int parkDistance, int side) {
        // locations
        Pose2d pickUp = side > 0 ? positions.RightConeStack:positions.LeftConeStack;
        Pose2d dropOff = Medium;

        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        // forward/backwards does not need to be reversed
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .forward(48)
                .strafeLeft(36*side)
                .build();

        TrajectorySequence drop = drive.trajectorySequenceBuilder(pickUp)
                .lineToLinearHeading(dropOff)
                .build();
        TrajectorySequence pickFromMedium = drive.trajectorySequenceBuilder(dropOff)
                .lineToLinearHeading(pickUp)
                .build();
        TrajectorySequence goToReady = drive.trajectorySequenceBuilder(positions.BottomHigh)
                // go to drop off, but except there is no heading
                .lineToConstantHeading(new Vector2d(24*side, -12))
                .build();
        TrajectorySequence park = drive.trajectorySequenceBuilder(pickFromMedium.end())
                .lineToLinearHeading(new Pose2d(parkDistance*side, -12, Math.toRadians(90)))
                .build();


        if (op.isStopRequested()) return;
        drive.followTrajectorySequence(trajSeq1);
        cone.drop = true;
        cone.align(IntakeSlideSubsystemAuto.LiftState.HIGH, false);
        drive.setPoseEstimate(positions.BottomHigh);
        drive.followTrajectorySequence(goToReady);
        for (int i = 0; i < 1; i++) {
            drive.followTrajectorySequence(pickFromMedium);
            cone.pickUpCone();
            drive.followTrajectorySequence(drop);
            cone.align(IntakeSlideSubsystemAuto.LiftState.MEDIUM, coneThere);
            coneThere = true;
        }
        drive.followTrajectorySequence(park);
        intakeSlide.runToREST();
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