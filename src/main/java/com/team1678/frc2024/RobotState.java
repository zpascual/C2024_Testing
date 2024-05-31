package com.team1678.frc2024;

import com.team1678.lib.geometry.MovingAverageTwist2d;
import com.team1678.lib.util.Translation2dHelper;
import com.team1678.lib.util.Twist2dHelper;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class RobotState {
    private static RobotState mInstance;
    //private Optional<VisionUpdate> mLatestVisionUpdate;
    //private UnscentedKalmanFilter<N2, N2, N2> mKalmanFilter;
    //private VisionPoseAcceptor mPoseAcceptor;
    private Pose2d mDisplayVisionPose;
    private Pose2d mSetpointPose;

    public Field2d mField2d;

    private boolean mHasBeenEnabled = false;

    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }

        return mInstance;
    }

    private static final int kObservationBufferSize = 50;

    /*
     * RobotState keeps track of the poses of various coordinate frames throughout
     * the match. A coordinate frame is simply a point and direction in space that
     * defines an (x,y) coordinate system. Transforms (or poses) keep track of the
     * spatial relationship between different frames.
     *
     * Robot frames of interest (from parent to child):
     *
     * 1. Field frame: origin is the right near corner of the playing field.
     *
     * 2. Odom frame: origin is where the robot is turned on.
     *
     * 3. Vehicle frame: origin is the center of the robot wheelbase, facing
     * forwards
     *
     * 4. Camera frame: origin is the center of the Limelight relative to the
     * turret.
     *
     * 5. Target frame: origin is the center of the vision target, facing outwards
     * along the normal.
     *
     * As a kinematic chain with 5 frames, there are 4 transforms of interest:
     *
     * 1. Field-to-odom: This is derived from a fused field-to-vehicle transform incorporating both vision and odometry.
     *
     * 2. Odom-to-vehicle: Tracked by integrating encoder and gyro odometry.
     *
     * 3. Vehicle-to-camera: This is a constant (per camera).
     *
     * 4. Camera-to-target: Measured by the vision system.
     */
    private Optional<Translation2d> initial_field_to_odom_ = Optional.empty();
    private InterpolatingTreeMap<Double, Pose2d> odom_to_vehicle_;
    private InterpolatingTreeMap<Double, Translation2d> field_to_odom_;

    private Twist2d vehicle_velocity_predicted_;
    private Twist2d vehicle_velocity_measured_;
    private MovingAverageTwist2d vehicle_velocity_measured_filtered_;

    private RobotState() {
        reset(0.0, new Pose2d());
    }


    public synchronized void reset(double start_time, Pose2d initial_odom_to_vehicle) {
        odom_to_vehicle_.put(start_time, initial_odom_to_vehicle);
        field_to_odom_.put(start_time, getInitialFieldToOdom().getTranslation());
        vehicle_velocity_predicted_ = new Twist2d();
        vehicle_velocity_measured_ = new Twist2d();
        vehicle_velocity_measured_filtered_ = new MovingAverageTwist2d(25);
        //mLatestVisionUpdate = Optional.empty();
        mDisplayVisionPose = new Pose2d();
        mSetpointPose = new Pose2d();
        //mPoseAcceptor = new VisionPoseAcceptor();

        mField2d = new Field2d();
        //mField2d.setRobotPose(Constants.kWidthField2d, Constants.kHeightField2d, new edu.wpi.first.math.geometry.Rotation2d(0));
        //mField2d.getObject("vision").setPose(Constants.kWidthField2d, Constants.kHeightField2d, new edu.wpi.first.math.geometry.Rotation2d(0));
        //mField2d.getObject("fused").setPose(Constants.kWidthField2d, Constants.kHeightField2d, new edu.wpi.first.math.geometry.Rotation2d(0));
    }

    public synchronized void reset() {
        reset(Timer.getFPGATimestamp(), new Pose2d());
    }

    /*
    public synchronized void resetKalmanFilters() {
        mKalmanFilter =
        new UnscentedKalmanFilter<>(
            Nat.N2(),
            Nat.N2(),
            (x, u) -> VecBuilder.fill(0.0, 0.0),
            (x, u) -> x,
            Constants.kStateStdDevs,
            Constants.kLocalMeasurementStdDevs, Constants.kLooperDt);

    }
    */

    public synchronized boolean getHasBeenEnabled() {
        return mHasBeenEnabled;
    }

    public synchronized void setHasBeenEnabled(boolean hasBeenEnabled) {
        mHasBeenEnabled = hasBeenEnabled;
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */
    public synchronized Pose2d getOdomToVehicle(double timestamp) {
        return odom_to_vehicle_.get(timestamp);
    }

    public synchronized Pose2d getLatestOdomToVehicle() {
        return odom_to_vehicle_.get(Timer.getFPGATimestamp());
    }

    public synchronized Pose2d getPredictedOdomToVehicle(double lookahead_time) {
        Twist2d scaledVehicleVelocityPredicted = new Twist2d(vehicle_velocity_predicted_.dx * lookahead_time, vehicle_velocity_predicted_.dy * lookahead_time, vehicle_velocity_predicted_.dtheta * lookahead_time);
        return getLatestOdomToVehicle().relativeTo(getLatestOdomToVehicle().exp(scaledVehicleVelocityPredicted));
    }

    public synchronized void addOdomToVehicleObservation(double timestamp, Pose2d observation) {
        odom_to_vehicle_.put(timestamp, observation);
    }

    public synchronized void addOdomObservations(double timestamp, Pose2d odom_to_robot, Twist2d measured_velocity, Twist2d predicted_velocity) {
        try {
           // mKalmanFilter.predict(VecBuilder.fill(0.0, 0.0), Constants.kLooperDt);
        } catch (Exception e) {
            DriverStation.reportError("QR Decomposition failed: ", e.getStackTrace());
        }
        addOdomToVehicleObservation(timestamp, odom_to_robot);

        vehicle_velocity_measured_ = measured_velocity;
        vehicle_velocity_measured_filtered_.add(vehicle_velocity_measured_);
        vehicle_velocity_predicted_ = predicted_velocity;
    }

    public synchronized Twist2d getPredictedVelocity() {
        return vehicle_velocity_predicted_;
    }

    public synchronized Twist2d getMeasuredVelocity() {
        return vehicle_velocity_measured_;
    }

    public synchronized Twist2d getSmoothedVelocity() {
        return vehicle_velocity_measured_filtered_.getAverage();
    }

    public synchronized Pose2d getDisplayVisionPose() {
        if (mDisplayVisionPose == null) {
            //return Pose2d.fromTranslation(new Translation2d(Constants.kOutofFrameValue, Constants.kOutofFrameValue)); //Out of frame
        }
        return mDisplayVisionPose;
    }

    /**
     * Return Initial Vision Offset for Pure Odometry Visualization Purposes
     */
    public synchronized Pose2d getInitialFieldToOdom() {
        return initial_field_to_odom_.map(translation2d -> new Pose2d(translation2d, new Rotation2d())).orElseGet(Pose2d::new);
    }

    public synchronized Translation2d getFieldToOdom(double timestamp) {
        if (initial_field_to_odom_.isEmpty()) return new Translation2d();
        return initial_field_to_odom_.get().unaryMinus().plus(field_to_odom_.get(timestamp));
    }

    public synchronized Translation2d getAbsoluteFieldToOdom(double timestamp) {
        return field_to_odom_.get(timestamp);
    }

    public synchronized Translation2d getLatestFieldToOdom() {
        return getFieldToOdom(Timer.getFPGATimestamp());
    }

    /**
     * Get Current Field to Vehicle using Filter Idea 1 (Offset Space) => Add the Offset outputted by the Filter to Current Odom
     * @param timestamp
     * @return
     */
    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        Pose2d odomToVehicle = getOdomToVehicle(timestamp);

        Translation2d fieldToOdom = getFieldToOdom(timestamp);
        return new Pose2d(fieldToOdom.plus(odomToVehicle.getTranslation()), odomToVehicle.getRotation());

    }

    public synchronized Pose2d getFieldToVehicleAbsolute(double timestamp) {
        var field_to_odom = initial_field_to_odom_.orElse(new Translation2d());
        Pose2d fieldToOdomPose = new Pose2d(field_to_odom, new Rotation2d());
        Transform2d newPoseTransform = new Transform2d(fieldToOdomPose, getFieldToVehicle(timestamp));
        return fieldToOdomPose.transformBy(newPoseTransform);
    }

    public synchronized Pose2d getLatestFieldToVehicle() {
        Pose2d odomToVehicle = getLatestOdomToVehicle();
        return new Pose2d(getLatestFieldToOdom().plus(odomToVehicle.getTranslation()), odomToVehicle.getRotation());
    }


    /*
    public synchronized Optional<VisionUpdate> getLatestVisionUpdate() {
        return mLatestVisionUpdate;
    }
    */

    // TODO: Refactor outputs to be logged
    public void outputToSmartDashboard() {
        SmartDashboard.putString("Robot Velocity", getMeasuredVelocity().toString());

        SmartDashboard.putString("Odom To Robot", getOdomToVehicle(Timer.getFPGATimestamp()).toString());

        SmartDashboard.putString("Field To Robot", getFieldToVehicle(Timer.getFPGATimestamp()).toString());

        SmartDashboard.putNumber("Field To Robot X", getFieldToVehicle(Timer.getFPGATimestamp()).getTranslation().getX());
        double fieldX = getFieldToVehicle(Timer.getFPGATimestamp()).getTranslation().getX();
        double odomX = getOdomToVehicle(Timer.getFPGATimestamp()).getTranslation().getX() + getInitialFieldToOdom().getTranslation().getX();

        double [] arr = new double[]{fieldX, odomX};
        SmartDashboard.putNumberArray("Poses", arr);
        SmartDashboard.putNumber("Field To Robot Y", getFieldToVehicle(Timer.getFPGATimestamp()).getTranslation().getY());
        SmartDashboard.putNumber("Field To Robot Theta", getFieldToVehicle(Timer.getFPGATimestamp()).getRotation().getDegrees());

        SmartDashboard.putNumber("Odom X", getOdomToVehicle(Timer.getFPGATimestamp()).getTranslation().getX());
        SmartDashboard.putNumber("Odom Y", getOdomToVehicle(Timer.getFPGATimestamp()).getTranslation().getY());
        SmartDashboard.putNumber("Odom Theta", getOdomToVehicle(Timer.getFPGATimestamp()).getRotation().getDegrees());

        SmartDashboard.putString("Field to Odom Offset", getLatestFieldToOdom().toString());
        SmartDashboard.putString("initial offset", initial_field_to_odom_.toString());
   }

   public synchronized void displayField() {
        double timestamp = Timer.getFPGATimestamp();
        var displayVisionPose = getDisplayVisionPose();
        var fusedPose = getFieldToVehicleAbsolute(timestamp);
        var setpointPose = mSetpointPose;
        if (DriverStation.getAlliance().toString().equals(DriverStation.Alliance.Blue.toString())) {
            mField2d.getObject("vision").setPose(displayVisionPose.getTranslation().getX(), displayVisionPose.getTranslation().getY(), new edu.wpi.first.math.geometry.Rotation2d(displayVisionPose.getRotation().getRadians()));
            mField2d.getObject("fused").setPose(fusedPose.getTranslation().getX(), fusedPose.getTranslation().getY(), new edu.wpi.first.math.geometry.Rotation2d(fusedPose.getRotation().getRadians()));
            mField2d.getObject("setpoint").setPose(setpointPose.getTranslation().getX(), setpointPose.getTranslation().getY(), new edu.wpi.first.math.geometry.Rotation2d(setpointPose.getRotation().getRadians()));
        } else {
            /*
            mField2d.getObject("vision").setPose(Constants.kWidthField2d - displayVisionPose.getTranslation().x(), Constants.kHeightField2d - displayVisionPose.getTranslation().y(), new edu.wpi.first.math.geometry.Rotation2d(displayVisionPose.getRotation().getRadians() + Math.PI));
            mField2d.getObject("fused").setPose(Constants.kWidthField2d - fusedPose.getTranslation().x(), Constants.kHeightField2d - fusedPose.getTranslation().y(), new edu.wpi.first.math.geometry.Rotation2d(fusedPose.getRotation().getRadians() + Math.PI));
            mField2d.getObject("setpoint").setPose(Constants.kWidthField2d - setpointPose.getTranslation().x(), Constants.kHeightField2d - setpointPose.getTranslation().y(), new edu.wpi.first.math.geometry.Rotation2d(setpointPose.getRotation().getRadians() + Math.PI));
             */
        }
        SmartDashboard.putData("field", mField2d);
    }

    public Pose2d getFieldToGoal() {
        return new Pose2d();
    }

    public void setDisplaySetpointPose(Pose2d setpoint) {
        mSetpointPose = setpoint;
    }
}