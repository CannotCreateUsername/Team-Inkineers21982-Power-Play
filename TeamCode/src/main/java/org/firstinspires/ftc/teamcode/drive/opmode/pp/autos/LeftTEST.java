package org.firstinspires.ftc.teamcode.drive.opmode.pp.autos;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Cone;
import org.firstinspires.ftc.teamcode.drive.intakeslide.IntakeSlideSubsystemAuto;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.pp.AutoInterface;
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

    LinearOpMode op;
    SampleMecanumDrive drive;
    IntakeSlideSubsystemAuto intakeSlide;
    Cone cone;
    AutoInterface positions = new AutoInterface();

    double widthOfTile = 23.5; 		//includes ½ of tile edge on both sides
    double widthOfTileEdge = 0.75;
    double lengthOfRobot =13.972;
    double widthOfRobot = 13.465;

    // Low Top
    float yTop = 48;
    float xTop = -12;

    // Low Beside
    float yBeside = 36;
    float xBeside = 0;

    public void init(SampleMecanumDrive d, IntakeSlideSubsystemAuto i, Cone c, LinearOpMode o, int side) {
        drive = d;
        intakeSlide = i;
        cone = c;
        op = o;

        Pose2d TopLow = new Pose2d(xTop, yTop, Math.toRadians(90));
        Pose2d BesideLow = new Pose2d(xBeside, yBeside, Math.toRadians(90));
        Pose2d TopMedium = new Pose2d(xTop+24, yTop, Math.toRadians(90));
        Pose2d BesideMedium = new Pose2d(xBeside+24, yBeside, Math.toRadians(90));
        Pose2d TopHigh = new Pose2d(xTop+48, yTop, Math.toRadians(90));
        Pose2d BesideHigh = new Pose2d(xBeside+48, yBeside, Math.toRadians(90));
        Pose2d ConeStack = new Pose2d(xTop, yTop, Math.toRadians(0));

    }

    public void followPath(int parkDistance) {
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
        drive.followTrajectorySequence(trajSeq1);
        cone.drop = true;
        runtime.reset();
        while (runtime.seconds() < 2) {
            // wait.. add telemetry here
            op.telemetry.addData("Waiting", "to align");
            op.telemetry.update();
        }
        cone.align(IntakeSlideSubsystemAuto.LiftState.MEDIUM, false);
        drive.followTrajectorySequence(park);
        intakeSlide.liftState = IntakeSlideSubsystemAuto.LiftState.REST;
        intakeSlide.run();
    }

    public void followPath2() {
        Pose2d pickUp = positions.LeftConeStack;
        Pose2d dropOff = positions.Medium;

        drive.setPoseEstimate(pickUp);

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(pickUp)
                .lineToLinearHeading(dropOff)
                .build();

        if (op.isStopRequested()) return;
        runtime.reset();
        drive.followTrajectorySequence(trajSeq1);
        intakeSlide.liftState = IntakeSlideSubsystemAuto.LiftState.REST;
        intakeSlide.run();
    }
    public void followPath3() {

    }
}