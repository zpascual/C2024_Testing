package com.team1678.lib.control;

import com.team1678.frc2024.Constants;
import com.team1678.frc2024.RobotState;
import com.team1678.lib.util.Stopwatch;
import com.team254.lib.util.SynchronousPIDF;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Controls overall swerve heading of the robot through motion profile.
 * <p>
 * All units are in degrees (for this class only) for easy integration with DPad
 */
public class SwerveHeadingController {
    private static SwerveHeadingController mInstance;
    public static SwerveHeadingController getInstance() {
        if (mInstance == null) {
            mInstance = new SwerveHeadingController();
        }

        return mInstance;
    }

    public enum HeadingControllerState {
        OFF, MOVE_SNAP, STATIONARY_SNAP,// for snapping to specific headings
        MAINTAIN, // maintaining current heading while driving
        POLAR_MAINTAIN, // for maintaining heading toward origin
        POLAR_SNAP, // for snapping heading toward origin
        TEMPORARILY_DISABLE, // basic...
    }

    private final SynchronousPIDF mPIDFController;
    private double mSetpoint = 0.0;
    private final Stopwatch disabledStopWatch;
    private final double disableTimeLength = 0.2; //ms

    private HeadingControllerState mHeadingControllerState = HeadingControllerState.OFF;

    private SwerveHeadingController() {
        mPIDFController = new SynchronousPIDF();
        disabledStopWatch = new Stopwatch();
    }

    public HeadingControllerState getState() {
        return mHeadingControllerState;
    }

    public void setState(HeadingControllerState state) {
        mHeadingControllerState = state;
    }

    /**
     * @param targetAngle Target angle in degrees
     */
    public void setTarget(double targetAngle) {
        mSetpoint = targetAngle;
    }

    public double getTargetSetpoint() {
        return mSetpoint;
    }

    public boolean isAtTarget() {
        return mPIDFController.onTarget(Constants.SwerveHeadingController.kErrorTolerance);
    }

    public void disable() {
        setState(HeadingControllerState.OFF);
    }

    public void temporarilyDisable() {
        disable();
        disabledStopWatch.start();
    }

    public void setMoveSnapTarget(double angle) {
        setTarget(angle);
        setState(HeadingControllerState.MOVE_SNAP);
    }

    public void setPolarSnapTarget(double angle) {
        setTarget(angle);
        setState(HeadingControllerState.POLAR_SNAP);
    }

    public void setMaintainTarget(double angle) {
        setTarget(angle);
        setState(HeadingControllerState.MAINTAIN);
    }

    public void setPolarMaintainTarget(double angle) {
        setTarget(angle);
        setState(HeadingControllerState.POLAR_MAINTAIN);
    }

    public void setStationarySnapTarget(double angle) {
        setTarget(angle);
        setState(HeadingControllerState.STATIONARY_SNAP);
    }

    /**
     * Should be called from a looper at a constant dt
     */
    public double update(double current_angle) {
        mPIDFController.setSetpoint(mSetpoint);
        double current_error = mSetpoint - current_angle;

        if (current_error > 180) {
            current_angle += 360;
        } else if (current_error < -180) {
            current_angle -= 360;
        }

        switch (mHeadingControllerState) {
            case OFF:
                return 0.0;
            case MOVE_SNAP, POLAR_SNAP:
                mPIDFController.setPID(Constants.SwerveHeadingController.kSnapKp, Constants.SwerveHeadingController.kSnapKi, Constants.SwerveHeadingController.kSnapKd);
                break;
            case MAINTAIN:
                mPIDFController.setPID(Constants.SwerveHeadingController.kMaintainKpHighVelocity, Constants.SwerveHeadingController.kMaintainKiHighVelocity, Constants.SwerveHeadingController.kMaintainKdHighVelocity);
                mPIDFController.setOutputRange(-1.0, 1.0);
                break;
            case POLAR_MAINTAIN:
                mPIDFController.setPID(Constants.SwerveHeadingController.kPolarMaintainKp, Constants.SwerveHeadingController.kPolarMaintainKi, Constants.SwerveHeadingController.kPolarMaintainKd);
                break;
            case TEMPORARILY_DISABLE:
                setTarget(current_angle);
                if (disabledStopWatch.getTime() >= disableTimeLength) {
                    setState(HeadingControllerState.MAINTAIN);
                    disabledStopWatch.reset();
                }
                break;
            case STATIONARY_SNAP:
                mPIDFController.setPID(Constants.SwerveHeadingController.kSnapStationaryKp, Constants.SwerveHeadingController.kSnapStationaryKi, Constants.SwerveHeadingController.kSnapStationaryKd);

        }

        return mPIDFController.calculate(current_angle);
    }
}