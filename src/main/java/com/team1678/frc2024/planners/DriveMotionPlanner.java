package com.team1678.frc2024.planners;

import java.util.Optional;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.team1678.frc2024.Constants;
import com.team1678.lib.control.ErrorTracker;
import com.team1678.lib.control.Lookahead;
import com.team1678.lib.util.Pose2dHelper;
import com.team1678.lib.util.Rotation2dHelper;
import com.team254.lib.util.SynchronousPIDF;
import com.team254.lib.util.Util;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveMotionPlanner {
    private static final double kMaxDx = 0.0127; // m
    private static final double kMaxDy = 0.0127; // m
    private static final double kMaxDTheta = Math.toRadians(1.0);
    private boolean useDefaultCook = true;
    private double defaultCook = 0.5;

    public enum FollowerType {
        PID,
        PURE_PURSUIT,
    }

    FollowerType mFollowerType = FollowerType.PURE_PURSUIT;

    public void setFollowerType(FollowerType type) {
        mFollowerType = type;
    }
    ChoreoTrajectory mCurrentTrajectory;
    public ChoreoTrajectory getTrajectory() {
        return mCurrentTrajectory;
    }
    public double getRemainingProgress() {
        if (mCurrentTrajectory != null) {
            return Math.max(0.0, mCurrentTrajectory.getTotalTime() - mSetpoint.timestamp);
        }
        return 0.0;
    }

    boolean mIsReversed = false;
    double mLastTime = Double.POSITIVE_INFINITY;
    public ChoreoTrajectoryState mLastSetpoint = null;
    public ChoreoTrajectoryState mSetpoint;
    Pose2d mError = new Pose2d();
    ErrorTracker mErrorTracker = new ErrorTracker(15 * 100);
    Translation2d mTranslationalError = new Translation2d();
    Rotation2d mPrevHeadingError = new Rotation2d();
    Pose2d mCurrentState = new Pose2d();
    double mCurrentTrajectoryLength = 0.0;
    double mTotalTime = Double.POSITIVE_INFINITY;
    double mStartTime = Double.POSITIVE_INFINITY;
    ChassisSpeeds mOutput = new ChassisSpeeds();
    Lookahead mSpeedLookahead = null;

    // PID controllers for path following
    SynchronousPIDF mXPIDF;
    SynchronousPIDF mYPIDF;
    SynchronousPIDF mHeadingPIDF;

    double mDt = 0.0;

    public double getMaxRotationSpeed() {
        final double kStartPoint = 0.1;
        final double kPivotPoint = 0.5;
        final double kEndPoint = 1.0; // 0.8
        final double kMaxSpeed = 1.0;
        double normalizedProgress = mLastSetpoint.timestamp / mCurrentTrajectoryLength;
        double scalar = 0.0;
        if (kStartPoint <= normalizedProgress && normalizedProgress <= kEndPoint) {
            if (normalizedProgress <= kPivotPoint) {
                scalar = (normalizedProgress - kStartPoint) / (kPivotPoint - kStartPoint);
            } else {
                scalar = 1.0 - ((normalizedProgress - kPivotPoint) / (kPivotPoint - kStartPoint));
            }
        }
        return  kMaxSpeed * scalar;
    }

    public DriveMotionPlanner() {
    }

    public void setTrajectory(ChoreoTrajectory trajectory) {
        mCurrentTrajectory = trajectory;
        mSetpoint = trajectory.getInitialState();
        mCurrentTrajectoryLength = trajectory.getTotalTime();
        for (ChoreoTrajectoryState state : mCurrentTrajectory.getSamples()) {
            var velocity = Math.sqrt(Math.pow(state.getChassisSpeeds().vxMetersPerSecond,2) + Math.pow(state.getChassisSpeeds().vyMetersPerSecond,2));
            if (velocity > Util.kEpsilon) {
                mIsReversed = false;
                break;
            } else if (velocity < -Util.kEpsilon) {
                mIsReversed = true;
                break;
            }
        }
    }

    public void reset() {
        mErrorTracker.reset();
        mTranslationalError = new Translation2d();
        mPrevHeadingError = new Rotation2d();
        mLastSetpoint = null;
        mOutput = new ChassisSpeeds();
        mLastTime = Double.POSITIVE_INFINITY;
    }

    protected ChassisSpeeds updatePIDChassis(ChassisSpeeds chassisSpeeds) {
        // Feedback on longitudinal error (distance).
        final double kPathk = 2.4;//2.4;/* * Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)*/;//0.15;
        final double kPathKTheta = 2.4;

        Twist2d pid_error = Pose2dHelper.log(mError);

        chassisSpeeds.vxMetersPerSecond =
                chassisSpeeds.vxMetersPerSecond + kPathk * pid_error.dx;
        chassisSpeeds.vyMetersPerSecond =
                chassisSpeeds.vyMetersPerSecond + kPathk * pid_error.dy;
        chassisSpeeds.omegaRadiansPerSecond =
                chassisSpeeds.omegaRadiansPerSecond + kPathKTheta * pid_error.dtheta;
        return chassisSpeeds;
    }

    protected ChassisSpeeds updatePurePursuit(Pose2d current_state) {
        double lookahead_time = Constants.PurePursuit.kPathLookaheadTime;
        final double kLookaheadSearchDt = 0.01;

        ChoreoTrajectoryState lookahead_state = mCurrentTrajectory.sample(lookahead_time);
        double actual_lookahead_distance = mSetpoint.getPose().minus(lookahead_state.getPose()).getTranslation().getNorm();
        double adaptive_lookahead_distance = mSpeedLookahead.getLookaheadForSpeed(new Translation2d(mSetpoint.velocityX, mSetpoint.velocityY).getNorm());

        //Find the Point on the Trajectory that is Lookahead Distance Away
        while (actual_lookahead_distance < adaptive_lookahead_distance &&
                getRemainingProgress() > lookahead_time) {
            lookahead_time += kLookaheadSearchDt;
            lookahead_state = mCurrentTrajectory.sample(lookahead_time);
            actual_lookahead_distance = mSetpoint.getPose().minus(lookahead_state.getPose()).getTranslation().getNorm();
        }

        //If the Lookahead Point's Distance is less than the Lookahead Distance transform it, so it is the lookahead distance away
        if (actual_lookahead_distance < adaptive_lookahead_distance) {
            double xTranslation = (mIsReversed ? -1.0 : 1.0) * (Constants.PurePursuit.kPathMinLookaheadDistance - actual_lookahead_distance);
            double yTranslation = 0.0;
            Translation2d lookahead_translation = lookahead_state.getPose().getTranslation().plus(new Translation2d(xTranslation, yTranslation));
            Rotation2d lookahead_rotation = new Rotation2d();

            lookahead_state = new ChoreoTrajectoryState(
                    lookahead_state.timestamp,
                    lookahead_translation.getX(),
                    lookahead_translation.getY(),
                    lookahead_state.heading,
                    lookahead_state.velocityX,
                    lookahead_state.velocityY,
                    lookahead_state.angularVelocity
            );
        }

        //Find the vector between robot's current position and the lookahead state
        Translation2d lookahead_translation = lookahead_state.getPose().getTranslation().minus(current_state.getTranslation());

        //Set the steering direction as the direction of the vector
        Rotation2d steeringDirection = lookahead_translation.getAngle();

        //Convert from field-relative steering direction to robot-relative
        steeringDirection = steeringDirection.rotateBy(Pose2dHelper.inverse(current_state).getRotation());

        //Use the Velocity Feedforward of the Closest Point on the Trajectory
        double normalizedSpeed = Math.abs(new Translation2d(mSetpoint.velocityX, mSetpoint.velocityY).getNorm()) / Constants.SwerveConfig.kMaxLinearVelocity;

        //The Default Cook is the minimum speed to use. So if a feedforward speed is less than defaultCook, the robot will drive at the defaultCook speed
        if(normalizedSpeed > defaultCook || mSetpoint.timestamp > (mCurrentTrajectoryLength / 2.0)){
            useDefaultCook = false;
        }
        if(useDefaultCook){
            normalizedSpeed = defaultCook;
        }

        //Convert the Polar Coordinate (speed, direction) into a Rectangular Coordinate (Vx, Vy) in Robot Frame
        final Translation2d steeringVector = new Translation2d(steeringDirection.getCos() * normalizedSpeed, steeringDirection.getSin() * normalizedSpeed);
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(steeringVector.getX() * Constants.SwerveConfig.kMaxLinearVelocity, steeringVector.getY() * Constants.SwerveConfig.kMaxLinearVelocity, 0.0);

        //Use the PD-Controller for To Follow the Time-Parametrized Heading
        final double kThetakP = 3.5;
        final double kThetakD = 0.0;
        final double kPositionkP = 2.0;

        chassisSpeeds.vxMetersPerSecond =
                chassisSpeeds.vxMetersPerSecond + kPositionkP * mError.getTranslation().getX();
        chassisSpeeds.vyMetersPerSecond =
                chassisSpeeds.vyMetersPerSecond + kPositionkP * mError.getTranslation().getY();
        chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond + (kThetakP * mError.getRotation().getRadians()) + kThetakD * ((mError.getRotation().getRadians() - mPrevHeadingError.getRadians()) / mDt);
        return chassisSpeeds;
    }

    public ChassisSpeeds update(double timestamp, Pose2d current_state) {
        if (mCurrentTrajectory == null) return new ChassisSpeeds();

        if (!Double.isFinite(mLastTime)) mLastTime = timestamp;
        mDt = timestamp - mLastTime;
        mLastTime = timestamp;
        ChoreoTrajectoryState samplePoint;
        mCurrentState = current_state;

        if (isDone()) {
            // Compute error in robot frame
            mPrevHeadingError = mError.getRotation();
            mError = Pose2dHelper.inverse(current_state).relativeTo(mSetpoint.getPose());
            mErrorTracker.addObservation(mError);

            if (mFollowerType == FollowerType.PID) {
                samplePoint = mCurrentTrajectory.sample(mDt);
                //RobotState.getInstance().setDisplaySetpointPose(new Pose2d(RobotState.getInstance().getFieldToOdom(timestamp), new Rotation2d()).relativeTo(sample_point.state().state().getPose()));
                mSetpoint = samplePoint;

                final double velocity_m = new Translation2d(mSetpoint.velocityX, mSetpoint.velocityY).getNorm();
                // Field relative
                Optional<Rotation2d> course = Optional.of(new Rotation2d(mSetpoint.velocityX, mSetpoint.velocityY));
                Rotation2d motion_direction = course.orElseGet(Rotation2d::new);
                // Adjust course by ACTUAL heading rather than planned to decouple heading and translation errors.
                motion_direction = Rotation2dHelper.inverse(current_state.getRotation()).rotateBy(motion_direction);

                var chassis_speeds = new ChassisSpeeds(
                        motion_direction.getCos() * velocity_m,
                        motion_direction.getSin() * velocity_m,
                        // Need unit conversion because Pose2dWithMotion heading rate is per unit distance.
                        velocity_m * mSetpoint.angularVelocity);
                // PID is in robot frame
                mOutput = updatePIDChassis(chassis_speeds);
            } else if (mFollowerType == FollowerType.PURE_PURSUIT) {
                double searchStepSize = 1.0;
                double previewQuantity = 0.0;
                double searchDirection = 1.0;
                double forwardDistance = distance(current_state, previewQuantity + searchStepSize);
                double reverseDistance = distance(current_state, previewQuantity - searchStepSize);
                searchDirection = Math.signum(reverseDistance - forwardDistance);
                while(searchStepSize > 0.001){
                    if(Util.epsilonEquals(distance(current_state, previewQuantity), 0.0, 0.001)) break;
                    while(/* next point is closer than current point */ distance(current_state, previewQuantity + searchStepSize*searchDirection) <
                            distance(current_state, previewQuantity)) {
                        /* move to next point */
                        previewQuantity += searchStepSize*searchDirection;
                    }
                    searchStepSize /= 10.0;
                    searchDirection *= -1;
                }
                samplePoint = mCurrentTrajectory.sample(previewQuantity);
                //RobotState.getInstance().setDisplaySetpointPose(new Pose2d(RobotState.getInstance().getFieldToOdom(timestamp), new Rotation2d()).relativeTo(sample_point.state().state().getPose()));
                mSetpoint = samplePoint;
                mOutput = updatePurePursuit(current_state);
            }
        } else {
            System.out.println("Motion Planner Done: Returning zero chassis speeds");
            mOutput = new ChassisSpeeds();
        }
        return mOutput;
    }

    public boolean isDone() {
        return mCurrentTrajectory == null || !(getRemainingProgress() == 0);
    }

    public synchronized Translation2d getTranslationalError() {
        return new Translation2d(
                mError.getTranslation().getX(),
                mError.getTranslation().getY());
    }

    public synchronized Rotation2d getHeadingError() {
        return mError.getRotation();
    }

    private double distance(Pose2d current_state, double additional_progress){
        return Pose2dHelper.distance(mCurrentTrajectory.sample(additional_progress).getPose(), current_state);
    }

    public synchronized ChoreoTrajectoryState getSetpoint() {
        return mSetpoint;
    }

    public synchronized ErrorTracker getErrorTracker() {
        return mErrorTracker;
    }
}