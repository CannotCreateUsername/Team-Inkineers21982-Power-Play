package org.firstinspires.ftc.teamcode.drive.opmode.pp.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.drive.Cone;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.intakeslide.IntakeSlide;
import org.firstinspires.ftc.teamcode.drive.intakeslide.IntakeSlideSubsystemAuto;
import org.firstinspires.ftc.teamcode.drive.opmode.pp.AutoInterface;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Cycle Medium LEFT", group = "Linear Opmode")
public class AutoMediumSolo extends LinearOpMode {

    enum DriveState {
        TRAJECTORY1,
        DROP_OFF,
        PARK,
        IDLE,
    }
    DriveState driveState = DriveState.IDLE;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean coneThere = false;

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

    // left side
    int iSide = 1;
    // Vuforia Variables
    int label = 0;
    int parkDistance = 1;
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
    private static final String VUFORIA_KEY =
            "AYP9k3r/////AAABmbsKp4S4+0RSpbJyMlJGbNQJWbthdpl1gIp8CO+DnDwIDkzifNXPuUMawrPbYmKwDfWtSi+PAKLOcvbHmHZxsTM24Sd32QsBy/RarvDqfIJgEIVDiUXpTlOvKCqFNCS5FGivU6Tz3C5FIhf5N/KapHhETsd2ExGtCtsZSE7QQw5SCjynKE+JvP/DnjZ8eBk6PYlS/TUdvQmonUSTkPwPCEXcL3HVO9Mw+QjvYT0eA93l7yn2NssK+37MjpJBn7kzME8FUmurwynqPJA5Ido5l/iafDl53Hndd+vl0H5ooXY0qVE1mc8HUK5lYoVXMygBDqa9Grkghg8bD791U09C20SnuKdwFCWH0Ic6zZUkeH9o";

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize stuffs
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        IntakeSlideSubsystemAuto intakeSlide = new IntakeSlideSubsystemAuto();
        intakeSlide.init(hardwareMap);
        Cone cone = new Cone();
        cone.init(drive, intakeSlide, hardwareMap, this);

        double rotation = iSide > 0 ? 0:180;

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

        // Vuforia
        initVuforia();
        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables targets1 = this.vuforia.loadTrackablesFromAsset("PowerPlay1");
        VuforiaTrackables targets2 = this.vuforia.loadTrackablesFromAsset("PowerPlay2");
        VuforiaTrackables targets3 = this.vuforia.loadTrackablesFromAsset("PowerPlay3");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets1);
        allTrackables.addAll(targets2);
        allTrackables.addAll(targets3);

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
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .setConstraints(SampleMecanumDrive.VEL_CONSTRAINT_FAST, SampleMecanumDrive.ACCEL_CONSTRAINT_FAST) // max speed
                .lineToLinearHeading(TopMid)
                .resetConstraints()
                .build();

        //Vufrofia
        targets1.activate();  // octopus
        targets2.activate(); // triangle
        targets3.activate(); // traffic

        boolean targetVisible = false;
        String targetName = "NOT FOUND";

        runtime.reset();
        while (!isStopRequested() && !opModeIsActive()) {
            if (!targetVisible) {
                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        targetVisible = true;
                        targetName = trackable.getName();
                        if (targetName == "PowerPlay2") {
                            label = 1;
//                                parkDistance = 48;
//                                parkDistance = 60;
                            parkDistance = -24;
                        } else if (targetName == "PowerPlay1") {
                            label = 2;
//                                parkDistance = 24;
//                                parkDistance = 36;
                            parkDistance = 0;
                        } else if (targetName == "PowerPlay3") {
                            label = 3;
//                                parkDistance = 1;
//                                parkDistance = 12;
                            parkDistance = 24;
                        }
                        break;
                    }
                }
            }
            intakeSlide.setIntakePosition(IntakeSlideSubsystemAuto.IntakeState.IN);
            telemetry.addData("Visible Target", targetName);
            telemetry.addData("Zone #", label);
            telemetry.addData("Park distance:", parkDistance);
            telemetry.update();
        }
        TrajectorySequence park = drive.trajectorySequenceBuilder(TopMid)
                .lineToLinearHeading(new Pose2d(parkDistance, yTopMid, Math.toRadians(90)))
                .build();

        if (isStopRequested()) return;
        runtime.reset();
        intakeSlide.runToMEDIUM();
        drive.followTrajectorySequence(traj1);
        BackIntoMidJunctionFromTop(drive, intakeSlide, cone); // drops the cone as well
        intakeSlide.runToPICKUP2();
        // change middle number for amount of times
        for (int i = 0; i < 2; i++) {
            drive.followTrajectorySequence(traj2);
            cone.simplePickUp2();
            drive.followTrajectorySequence(traj3);
            BackIntoMidJunctionFromTop(drive, intakeSlide, cone);
            intakeSlide.runToPICKUP2();
        }
        // tells intake to run to original rest state (and not cone stack)
        intakeSlide.stack = false;
        intakeSlide.runToREST();
        drive.followTrajectorySequence(park);
    }

    private void BackIntoMidJunctionFromTop(SampleMecanumDrive d, IntakeSlideSubsystemAuto i, Cone c) {
        SampleMecanumDrive drive = d;
        IntakeSlideSubsystemAuto intakeSlide = i;
        Cone cone = c;
        drive.setPoseEstimate(TopMid);
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(TopMid)
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .setConstraints(SampleMecanumDrive.VEL_CONSTRAINT_HALF, SampleMecanumDrive.ACCEL_CONSTRAINT_HALF) // max speed
                .lineToLinearHeading(new Pose2d(xTopMid * iSide, yTopMid-distanceBackIntoJunction, Math.toRadians(90)))
                .resetConstraints()
                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .lineToLinearHeading(TopMid)
                .build();

        if (isStopRequested()) return;
        runtime.reset();
        intakeSlide.runToMEDIUM();
        drive.followTrajectorySequence(traj1);
        cone.simpleDropOff();
        drive.followTrajectorySequence(traj2);
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
}