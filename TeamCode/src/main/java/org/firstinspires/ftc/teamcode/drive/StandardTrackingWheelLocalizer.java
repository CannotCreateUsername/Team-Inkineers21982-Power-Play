package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */

/**
 *  IY
 *  Tuning guide: https://learnroadrunner.com/dead-wheels.html#tuning-three-wheel
 *  sample of this file with three wheel tracking - https://gist.github.com/NoahBres/9b9710eaa9f9fd23efa30a16de0f610e
 */

@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    // IY: OpenOdometry - Rev Through Bore Encoder 8192 Counts per Revolution  ref - https://www.revrobotics.com/rev-11-1271/
    public static double TICKS_PER_REV = 8192;
    // IY: OpenOdometry 35mm diameter wheel ; ref: https://www.rotacaster.com.au/shop-product/robotic-wheels/rotacaster-35mm-double--solid--roller--abs-body-8-2mm-keyed-bore-no-bushing
    public static double WHEEL_RADIUS = 0.6889764; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    //IY: first measure the distance and forward offset
    // ref: https://learnroadrunner.com/dead-wheels.html#three-wheel-odometry
    // IY: later on, tune the LATERAL_DISTANCE using TrackingWheelLateralDistanceTuner
    // ref: https://learnroadrunner.com/dead-wheels.html#tuning-three-wheel
    public static double LATERAL_DISTANCE = 15.9; // 16.34;  in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -6.5; // in; offset of the lateral wheel

    // IY , using localizationtest , the measured X distance is 87.66.
    public static double X_MULTIPLIER = 1.0266940; // 90/87.66 // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.0266940; ; // Multiplier in the Y direction

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        /**
         *  IY:
         *    Discord information:
         *    https://discord.com/channels/225450307654647808/225450307654647808/1007732261770252469
         *    Try to avoid plugging odometry pods that use the REV through bore encoder or other high encoder tick (>4000 counts per rev) encoders into ports 1 and 2 on the control or expansion hubs, as these appear to be run in software instead of hardware and can miss ticks at high speed. If unavoidable, plugging the strafe pod into port 1 or 2 and the forward pods into 0 and 3 is preferable. Essentially, at speeds above 35-40 inches per second (achievable with 19.2:1 gobilda mecanum drivetrains), odometry pods that use through bore encoders and 35mm omni wheels will run too fast for encoders plugged into motor port 1 and motor port 2 to properly read the encoder. However, motor port 0 and motor port 3 encoder ports will still track fine even at very high speeds.
         *     left pod: 0 = left front
         *     right pod : 3 = right back
         *     horizontal pod: 1 = right front
         */
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "left_front_drive"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "right_back_drive"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "right_front_drive"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        // to verify the deirection: run the localization test
        // move forward : X will increase
        // move to right: y will decrease
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        // frontEncoder.setDirection(Encoder.Direction.REVERSE);

    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(

                /* IY:
                If your encoder velocity exceeds 32767 counts per second, it will cause an integer overflow when calling getVelocity(). This is because the Rev Hub firmware sends the velocity data using 16 bit signed integers rather than 32 bit. Due to the Rev Through Bore encoders' absurdly high CPR, this happens at around 4 rounds per second. Or only 25 inches per second with 2 inch diameter wheels.
                ref: https://learnroadrunner.com/dead-wheels.html#three-wheel-odometry
                 */
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
//                encoderTicksToInches(leftEncoder.getRawVelocity()) * X_MULTIPLIER,
//                encoderTicksToInches(rightEncoder.getRawVelocity()) * X_MULTIPLIER,
//                encoderTicksToInches(frontEncoder.getRawVelocity()) * Y_MULTIPLIER
        );
    }
}
