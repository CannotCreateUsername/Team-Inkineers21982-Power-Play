package org.firstinspires.ftc.teamcode.drive.opmode.pp.autos;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Cone;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.intakeslide.IntakeSlide;
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
    
    int iSide;

    public void init(SampleMecanumDrive d, IntakeSlideSubsystemAuto i, Cone c, LinearOpMode o, int startSide) {
        drive = d;
        intakeSlide = i;
        cone = c;
        op = o;
        iSide = startSide;
        double rotation = iSide > 0 ? 0:180;

        // junctions
        Start2 = new Pose2d(-34*iSide, -62, Math.toRadians(90));
        Low = new Pose2d(-42*iSide, -12, Math.toRadians(90));
        Medium = new Pose2d(-0*iSide, -12, Math.toRadians(90));
        High = new Pose2d(-0*iSide, -12, Math.toRadians(-90));
        BottomHigh = new Pose2d(0, -12,  Math.toRadians(90));

        Start = new Pose2d(iSide * xStart, yStart , Math.toRadians(90));
        ConeStack = new Pose2d(iSide * xConeStack + (1 * iSide), yConeStack, Math.toRadians(rotation));
        //ConeStack = new Pose2d(iSide * xConeStack - 2, yConeStack, Math.toRadians(0));
        TopLow = new Pose2d(iSide * xTopLow, yTopLow, Math.toRadians(90));
        TopMid = new Pose2d(iSide * xTopMid, yTopMid, Math.toRadians(90));
        TopHigh = new Pose2d(iSide * xTopHigh, yTopHigh, Math.toRadians(90));
        BesideLow = new Pose2d(iSide * xBesideLow, yBesideLow, Math.toRadians(90));
        BesideMid = new Pose2d(iSide * xBesideMid, yBesideMid, Math.toRadians(90));
        BesideHigh = new Pose2d(iSide * xBesideHigh, yBesideHigh, Math.toRadians(90));

        TopLeftMid = new Pose2d(iSide * xBesideLow, yTopMid, Math.toRadians(90));
        TopLeftHigh = new Pose2d(iSide * xBesideMid, yTopHigh, Math.toRadians(90));
        TopRightHigh = new Pose2d(iSide * xBesideHigh, yTopHigh, Math.toRadians(90));
        LeftMiddleArenaHigh = new Pose2d(iSide * xBesideLow, yBesideLow+widthOfTile, Math.toRadians(90));
        RightMiddleArenaHigh = new Pose2d(iSide *  xBesideMid, yBesideMid+widthOfTile, Math.toRadians(90));

    }

    public void followPath(int parkDistance, int side) {
        // locations
        Pose2d pickUp = side > 0 ? positions.LeftConeStack:positions.RightConeStack;
        Pose2d dropOff = Medium;

        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        // forward/backwards does not need to be reversed
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .forward(48)
                .strafeLeft(-36*side)
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
        while (runtime.seconds() < 2) {
            // wait for slides to finish running to rest since it runs async
        }
    }

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
                .setConstraints(SampleMecanumDrive.VEL_CONSTRAINT_FAST, SampleMecanumDrive.ACCEL_CONSTRAINT_FAST) // max speed
                .lineToLinearHeading(ConeStack)
                .resetConstraints()
                .build();
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(ConeStack)
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .setConstraints(SampleMecanumDrive.VEL_CONSTRAINT_FAST, SampleMecanumDrive.ACCEL_CONSTRAINT_FAST) // max speed
                .lineToLinearHeading(TopHigh)
                .resetConstraints()
                .build();
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(TopHigh)
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .setConstraints(SampleMecanumDrive.VEL_CONSTRAINT_MEDIUM, SampleMecanumDrive.ACCEL_CONSTRAINT_MEDIUM) // max speed
                .lineToLinearHeading(ConeStack)
                .resetConstraints()
                .build();
        TrajectorySequence park = drive.trajectorySequenceBuilder(ConeStack)
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .setConstraints(SampleMecanumDrive.VEL_CONSTRAINT_MEDIUM, SampleMecanumDrive.ACCEL_CONSTRAINT_MEDIUM) // max speed
                .lineToLinearHeading(new Pose2d(parkDistance, yTopMid, Math.toRadians(90)))
                .build();

        if (op.isStopRequested()) return;
        runtime.reset();
        intakeSlide.runToMEDIUM();
        drive.followTrajectorySequence(traj1);
        BackIntoMidJunctionFromTop(); // drops the cone as well
        intakeSlide.runToPICKUP2();
        drive.followTrajectorySequence(traj2);
        // change middle number for amount of times
        for (int i = 0; i < 1; i++) {
            cone.simplePickUp();
            drive.followTrajectorySequence(traj3);
            BackIntoMidJunctionFromTop2();
            intakeSlide.runToPICKUP2();
            drive.followTrajectorySequence(traj4);
        }
        cone.simplePickUp();
        // tells intake to run to original rest state (and not cone stack)
        intakeSlide.stack = false;
        intakeSlide.runToREST();
        drive.followTrajectorySequence(park);
        // tells intake to run to original rest state (and not cone stack)
        intakeSlide.stack = false;
        intakeSlide.runToREST();
        runtime.reset();
        while (runtime.seconds() < 2) {
            // wait for slides to finish running to rest since it runs async
        }
    }
    private void BackIntoMidJunctionFromTop() {
            drive.setPoseEstimate(TopMid);
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(TopMid)
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .setConstraints(SampleMecanumDrive.VEL_CONSTRAINT_HALF, SampleMecanumDrive.ACCEL_CONSTRAINT_HALF) // max speed
                .lineToLinearHeading(new Pose2d(xTopMid*iSide, yTopMid-distanceBackIntoJunction, Math.toRadians(90)))
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
    private void BackIntoMidJunctionFromTop2() {
        drive.setPoseEstimate(TopHigh);
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(TopHigh)
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .setConstraints(SampleMecanumDrive.VEL_CONSTRAINT_HALF, SampleMecanumDrive.ACCEL_CONSTRAINT_HALF) // max speed
                .lineToLinearHeading(new Pose2d(xTopHigh*iSide, yTopHigh-distanceBackIntoJunction, Math.toRadians(90)))
                .resetConstraints()
                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .lineToLinearHeading(TopHigh)
                .build();

        if (op.isStopRequested()) return;
        runtime.reset();
        intakeSlide.runToHIGH();
        drive.followTrajectorySequence(traj1);
        cone.simpleDropOff();
        drive.followTrajectorySequence(traj2);
    }
}
