package org.firstinspires.ftc.teamcode.drive.opmode.pp.autos;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Cone;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.constants.DriveConstants;
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

    double widthOfTile = 23.5; 		//includes 0.5 of tile edge on both sides
    double widthOfTileEdge = 0.75;
    double lengthOfRobot = 13.972;
    double widthOfRobot = 13.465;

    double yStartCenterOffset = 0.5 * widthOfTile + 0.5 * lengthOfRobot - lengthOfRobot  - widthOfTileEdge;
    //Distance from back of robot to center of cone guide
    //Math: diameterOfGear + radiusOfCone + yIntakeEdgeOffset = lengthOfConeGuide
    float lengthOfConeGuide = 5;
    float diameterOfCone = 4;

    double xStart = 0;
    double yStart = 0;

    double xBesideLow = xStart;
    double yBesideLow = yStart + 1.5 * widthOfTile + yStartCenterOffset;
    double xTopMid = xStart + 0.5 * widthOfTile;
    double yTopMid = yBesideLow + 0.5 * widthOfTile;
    double xConeStack = xStart - 1.5 * widthOfTile + 0.5 * widthOfRobot + 0.5 * widthOfTileEdge;
    double yConeStack = yTopMid;
    double xTopLow = xTopMid - widthOfTile;
    double yTopLow = yTopMid;
    double xTopHigh = xTopMid + widthOfTile;
    double yTopHigh = yTopMid;
    double xBesideMid = xStart + widthOfTile;
    double yBesideMid = yBesideLow;
    double xBesideHigh = xBesideMid + widthOfTile;
    double yBesideHigh = yBesideLow;

    //Should be about 7.764”
    double distanceBackIntoJunction = lengthOfConeGuide + ((widthOfTile - 0.5*diameterOfCone) - (0.5*widthOfTile + 0.5*lengthOfRobot));


    public Pose2d Start2 = positions.Start;
    public Pose2d Low = positions.Low;
    public Pose2d Medium = positions.Medium;
    public Pose2d High = positions.High;
    public Pose2d BottomHigh = positions.BottomHigh;

    Pose2d Start;
    Pose2d ConeStack;
    Pose2d TopLow;
    Pose2d TopMid;
    Pose2d TopHigh;
    Pose2d BesideLow;
    Pose2d BesideMid;
    Pose2d BesideHigh;

    Pose2d TopLeftMid;
    Pose2d TopLeftHigh;
    Pose2d TopRightHigh;
    Pose2d LeftMiddleArenaHigh;
    Pose2d RightMiddleArenaHigh;

    public void init(SampleMecanumDrive d, IntakeSlideSubsystemAuto i, Cone c, LinearOpMode o, int startSide) {
        drive = d;
        intakeSlide = i;
        cone = c;
        op = o;

        // junctions
        Start2 = new Pose2d(-34*startSide, -62, Math.toRadians(90));
        Low = new Pose2d(-42*startSide, -12, Math.toRadians(90));
        Medium = new Pose2d(-0*startSide, -12, Math.toRadians(90));
        High = new Pose2d(-0*startSide, -12, Math.toRadians(-90));
        BottomHigh = new Pose2d(0, -12,  Math.toRadians(90));

        Start = new Pose2d(startSide * xStart, yStart , Math.toRadians(90));
        ConeStack = new Pose2d(startSide * xConeStack + 0.75, yConeStack, Math.toRadians(0));
        //ConeStack = new Pose2d(startSide * xConeStack - 2, yConeStack, Math.toRadians(0));
        TopLow = new Pose2d(startSide * xTopLow, yTopLow, Math.toRadians(90));
        TopMid = new Pose2d(startSide * xTopMid, yTopMid, Math.toRadians(90));
        TopHigh = new Pose2d(startSide * xTopHigh, yTopHigh, Math.toRadians(90));
        BesideLow = new Pose2d(startSide * xBesideLow, yBesideLow, Math.toRadians(90));
        BesideMid = new Pose2d(startSide * xBesideMid, yBesideMid, Math.toRadians(90));
        BesideHigh = new Pose2d(startSide * xBesideHigh, yBesideHigh, Math.toRadians(90));

        TopLeftMid = new Pose2d(startSide * xBesideLow, yTopMid, Math.toRadians(90));
        TopLeftHigh = new Pose2d(startSide * xBesideMid, yTopHigh, Math.toRadians(90));
        TopRightHigh = new Pose2d(startSide * xBesideHigh, yTopHigh, Math.toRadians(90));
        LeftMiddleArenaHigh = new Pose2d(startSide * xBesideLow, yBesideLow+widthOfTile, Math.toRadians(90));
        RightMiddleArenaHigh = new Pose2d(startSide *  xBesideMid, yBesideMid+widthOfTile, Math.toRadians(90));

    }

    /**
     * Uses field centric coordinate system
     * @param parkDistance
     * @param side
     */
    public void followPath(int parkDistance, int side) {
        // locations
        Pose2d pickUp = side > 0 ? positions.LeftConeStack:positions.RightConeStack;
        Pose2d dropOff = Medium;

        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        // forward/backwards does not need to be reversed
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .setConstraints(SampleMecanumDrive.VEL_CONSTRAINT_MEDIUM, SampleMecanumDrive.ACCEL_CONSTRAINT_MEDIUM) // max speed
                .forward(48)
                .strafeLeft(-10*side)
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
        intakeSlide.runToREST();
    }

    /**
     * Uses robot centric coordinate system
     * @param parkDistance
     */
    public void followPath2(int parkDistance) {
        drive.setPoseEstimate(Start);
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(Start)
                .lineToLinearHeading(TopLeftMid)
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .setConstraints(SampleMecanumDrive.VEL_CONSTRAINT_HALF, SampleMecanumDrive.ACCEL_CONSTRAINT_HALF) // max speed
                .lineToLinearHeading(TopMid)
                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(TopMid)
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .setConstraints(SampleMecanumDrive.VEL_CONSTRAINT_MEDIUM, SampleMecanumDrive.ACCEL_CONSTRAINT_MEDIUM) // max speed
                .lineToLinearHeading(ConeStack)
                .resetConstraints()
                .build();
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .setConstraints(SampleMecanumDrive.VEL_CONSTRAINT_MEDIUM, SampleMecanumDrive.ACCEL_CONSTRAINT_MEDIUM) // max speed
                .lineToLinearHeading(TopMid)
                .resetConstraints()
                .build();
        TrajectorySequence park = drive.trajectorySequenceBuilder(TopMid)
                .lineToLinearHeading(new Pose2d(parkDistance, yTopMid, Math.toRadians(90)))
                .build();

        if (op.isStopRequested()) return;
        runtime.reset();
        intakeSlide.runToMEDIUM();
        drive.followTrajectorySequence(traj1);
        BackIntoMidJunctionFromTop(); // drops the cone as well
        intakeSlide.runToPICKUP2();
        // change middle number for amount of times
        for (int i = 0; i < 2; i++) {
            drive.followTrajectorySequence(traj2);
            cone.simplePickUp();
            drive.followTrajectorySequence(traj3);
            BackIntoMidJunctionFromTop();
            intakeSlide.runToPICKUP2();
        }
        drive.followTrajectorySequence(park);
        intakeSlide.liftState = IntakeSlideSubsystemAuto.LiftState.REST;
        intakeSlide.run();
    }
    private void BackIntoMidJunctionFromTop() {
        drive.setPoseEstimate(TopMid);
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(TopMid)
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .setConstraints(SampleMecanumDrive.VEL_CONSTRAINT_HALF, SampleMecanumDrive.ACCEL_CONSTRAINT_HALF) // max speed
                .lineToLinearHeading(new Pose2d(xTopMid, yTopMid-distanceBackIntoJunction, Math.toRadians(90)))
                .resetConstraints()
                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .lineToLinearHeading(TopMid)
                .build();

        if (op.isStopRequested()) return;
        runtime.reset();
        intakeSlide.runToMEDIUM();
        drive.followTrajectorySequence(traj1);
        cone.simpleDropOff();
        drive.followTrajectorySequence(traj2);
    }
}