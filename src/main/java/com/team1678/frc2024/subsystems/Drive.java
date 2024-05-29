package com.team1678.frc2024.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1678.frc2024.Constants;
import com.team1678.frc2024.Ports;
import com.team1678.frc2024.Settings;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.frc2024.planners.DriveMotionPlanner;
import com.team1678.frc2024.subsystems.swerve.CoaxialSwerveModule;
import com.team1678.frc2024.subsystems.swerve.SwerveModule;
import com.team1678.lib.control.SwerveHeadingController;
import com.team1678.lib.drivers.gyros.Gyro;
import com.team1678.lib.drivers.gyros.Pigeon2IMU;
import com.team1678.lib.kinematics.SwerveInverseKinematics;
import com.team1678.lib.util.Rotation2dHelper;
import com.team1678.lib.util.Translation2dHelper;
import com.team1678.lib.util.Util;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.List;

public class Drive implements ISubsystem {

    private static Drive instance = null;
    public static Drive getInstance() {
        return instance == null ? new Drive() : instance;
    }

    public enum ControlState {
        NEUTRAL, MANUAL, POSITION, ROTATION, DISABLED, TRAJECTORY, VELOCITY, VISION
    }
    private ControlState currentState = ControlState.NEUTRAL;

    public ControlState getState() {
        return currentState;
    }

    private void setState(ControlState desiredState) {
        currentState = desiredState;
    }

    public boolean isTracking() {
        return getState() == ControlState.VISION;
    }

    public SwerveModule frontRight, frontLeft, rearLeft, rearRight;
    private final List<SwerveModule> modules;
    private SwerveDriveKinematics kinematics;
    Gyro gyro;
    SwerveHeadingController headingController = SwerveHeadingController.getInstance();
    public void temporarilyDisableHeadingController() {
        headingController.temporarilyDisable();
    }

    public Rotation2d getTargetHeading() {
        return new Rotation2d(headingController.getTargetSetpoint());
    }

    boolean needsToNotifyDrivers = false;
    public boolean needsToNotifyDrivers() {
        if (needsToNotifyDrivers) {
            needsToNotifyDrivers = false;
            return true;
        }
        return false;
    }

    Pose2d robotPose;
    public Pose2d getRobotPose(){
        return robotPose;
    }
    double distanceTraveled;
    double currentVelocity = 0.0;
    double lastUpdateTimestamp = 0.0;

    SwerveDrivePoseEstimator poseEstimator;
    private boolean isPoseEstimatorInitialized = false;

    boolean modulesReady = false;
    boolean alwaysConfigModules = false;
    boolean moduleConfigRequested = false;
    public void requireModuleConfig() {
        modulesReady = false;
    }
    public void alwaysConfigModules() {
        alwaysConfigModules = true;
    }

    Pose2d startingPose = new Pose2d();
    public void setStartingPose(Pose2d startingPose) {
        this.startingPose = startingPose;
    }

    DriveMotionPlanner motionPlanner;

    public double getPlannerRemainingProgress() {
        if (motionPlanner != null && getState() == ControlState.TRAJECTORY) {
            return motionPlanner.getIterator().getRemainingProgress() / (motionPlanner.getIterator().getProgress() + motionPlanner.getIterator().getRemainingProgress());
        }
        return 0.0;
    }

    double rotationScalar;
    double trajectoryStartTime = 0.0;
    Translation2d lastTrajectoryVector = new Translation2d();
    boolean hasStartedTrjectory = false;
    boolean hasFinishedTrajectory = false;
    public boolean hasFinishedTrajectory() {
        return hasFinishedTrajectory;
    }

    /**
     * 			   	  Front
     *      _________________________
     * 	   | M1 |               | M0 |
     * 	   |____|               |____|
     * 	   |                         |
     * 	   |                         |
     * 	   |                         |
     * 	   |                         |
     * 	   |                         |
     * 	   |____                _____|
     * 	   | M2 |               | M3 |
     *     |____|_______________|____|
     *
     */
    private Drive() {
        frontRight = new CoaxialSwerveModule(0, Constants.SwerveModules.frontRight.kAngleOffset, false, SwerveModule.EncoderType.CANCoder, Constants.SwerveModules.frontRight.kAzmithMotorInfo, Constants.SwerveModules.frontRight.kDriveMotorInfo);
        frontLeft = new CoaxialSwerveModule(0, Constants.SwerveModules.frontLeft.kAngleOffset, false, SwerveModule.EncoderType.CANCoder, Constants.SwerveModules.frontLeft.kAzmithMotorInfo, Constants.SwerveModules.frontLeft.kDriveMotorInfo);
        rearRight = new CoaxialSwerveModule(0, Constants.SwerveModules.rearRight.kAngleOffset, false, SwerveModule.EncoderType.CANCoder, Constants.SwerveModules.rearRight.kAzmithMotorInfo, Constants.SwerveModules.rearRight.kDriveMotorInfo);
        rearLeft = new CoaxialSwerveModule(0, Constants.SwerveModules.rearLeft.kAngleOffset, false, SwerveModule.EncoderType.CANCoder, Constants.SwerveModules.rearLeft.kAzmithMotorInfo, Constants.SwerveModules.rearLeft.kDriveMotorInfo);

        modules = Arrays.asList(frontRight, frontLeft, rearRight, rearLeft);
        gyro = Pigeon2IMU.createRealorSimulatedGyro(Ports.PIGEON);
        motionPlanner = new DriveMotionPlanner();

        kinematics = new SwerveDriveKinematics(
                Constants.SwerveModules.frontRight.kVehicleToModule,
                Constants.SwerveModules.frontLeft.kVehicleToModule,
                Constants.SwerveModules.rearLeft.kVehicleToModule,
                Constants.SwerveModules.rearRight.kVehicleToModule
        );

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), getModulePositions(), new Pose2d(), VecBuilder.fill(0.1, 0.1, 0.1), VecBuilder.fill(0.1, 0.1, 0.1));
        robotPose = poseEstimator.getEstimatedPosition();
        distanceTraveled = 0.0;
    }

    public SwerveModulePosition[] getModulePositions() {
        return modules.stream().map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
    }

    public void setDriveNeutralMode(NeutralModeValue mode){
        modules.forEach(m -> m.setDriveNeutralMode(mode));
    }
    public void setAzmithNeutralmode(NeutralModeValue mode){
        modules.forEach(m -> m.setAzmithNeutralMode(mode));
    }

    /*
     * Teleop Driving Vars
     */

    private Translation2d translationVector = new Translation2d();
    private double rotationalInput = 0.0;
    private Translation2d lastDriveVector = new Translation2d();
    private final Translation2d rotationalVector = new Translation2d();
    private double lowPowerScalar = 0.6; // percent (0-1)
    private double maxRotSpeedScalar = 0.8; // percent (0-1)
    private double maxSpeedScalar = 1.0; // percent (0-1)
    private void setMaxSpeedScalar(double scalar) {maxSpeedScalar = scalar;}
    private boolean isTranslationStickReset = false;
    private SwerveInverseKinematics inverseKinematics = new SwerveInverseKinematics(Constants.SwerveModules.kModulePositions);
    private boolean isRobotCentric = false;
    public boolean getIsRobotCentric() {
        return isRobotCentric;
    }

    public void setIsRobotCentric(boolean isRobotCentric) {
        this.isRobotCentric = isRobotCentric;
    }

    public void sendInput(double x, double y, double rotate, boolean isLowPower) {
        Translation2d translationalInput = new Translation2d(x, y);
        double inputMagnitude = translationalInput.getNorm();

        /**
         * Used to snap the translational input to the nearest pole within a certain angular threshold.
         * Accounts for sloppy driver input and smoothens out sweeping the stick at full speed
         */
        double inputThresholdAngle = Math.toRadians(10);
        Rotation2d translationInputAngle = translationalInput.getAngle();
        Rotation2d inputNearestPoleAngleDist = translationalInput.getAngle().rotateBy(Rotation2dHelper.inverse(Rotation2dHelper.nearestPole(translationalInput.getAngle())));
        if (Math.abs(inputNearestPoleAngleDist.getRadians() - translationInputAngle.getRadians()) < inputThresholdAngle) {
            translationalInput = new Translation2d(inputNearestPoleAngleDist.getCos() * inputMagnitude, inputNearestPoleAngleDist.getSin() * inputMagnitude);
        }


        /**
         * Scale x and y by applying a power to the magnitude of the vector.
         * This makes the controls less sensitive at the low end.
         */
        double translationDeadband = 0.025;
        inputMagnitude = Util.scaledDeadband(inputMagnitude, 1.0, translationDeadband);
        final double power = (isLowPower) ? 2.0 : 1.5;
        inputMagnitude = Math.pow(inputMagnitude, power);
        inputMagnitude = Util.deadBand(inputMagnitude, 0.05);
        translationalInput = new Translation2d(inputMagnitude, translationalInput.getAngle());

        /**
         * Scale rotational input and apply a power to the input
         * to make the rotational controls less sensitive
         */
        double rotationalDeadband = 0.1;
        rotate = Util.scaledDeadband(rotate, 1.0, rotationalDeadband);
        rotate = Math.signum(rotate) * Math.pow(Math.abs(rotate), 1.75);

        translationalInput = Translation2dHelper.scale(translationalInput, maxSpeedScalar);
        rotate *= maxSpeedScalar;

        translationVector = translationalInput;


        /**
         * Apply low power scalar
         */
        if (isLowPower) {
            translationVector = Translation2dHelper.scale(translationVector, lowPowerScalar);
            rotate *= lowPowerScalar;
        } else {
            rotate *= maxRotSpeedScalar;
        }

        /**
         * Check if the rotation magnitude is not zero. If the magnitude isn't zero the heading controller
         * gets disabled. Final check is to make sure that if the input is zero, but the previous rotational magnitude
         * is not zero, it will only temporarily disable the heading controller to come to a controlled stop.
         */
        if (!Util.epsilonEquals(rotate, 0.0)) {
            headingController.disable();
        } else if (Util.epsilonEquals(rotate, 0.0) && !Util.epsilonEquals(rotationalInput, 0.0)) {
            headingController.temporarilyDisable();
        }

        rotationalInput = rotate;

        if (Util.epsilonEquals(translationalInput.getNorm(), 0.0) && !isTranslationStickReset) {
            isTranslationStickReset = true;
        }

        if (!Util.epsilonEquals(translationalInput.getNorm(), 0.0)) {
            if (headingController.getState() == SwerveHeadingController.HeadingControllerState.MOVE_SNAP ||
                headingController.getState() == SwerveHeadingController.HeadingControllerState.MAINTAIN ||
                headingController.getState() == SwerveHeadingController.HeadingControllerState.POLAR_SNAP ||
                headingController.getState() == SwerveHeadingController.HeadingControllerState.POLAR_MAINTAIN)
                headingController.setMaintainTarget(headingController.getTargetSetpoint());
            if (isTracking() || getState() == ControlState.POSITION) {
                if (Math.hypot(x, y) >= 0.5 && isTranslationStickReset) {
                    isTranslationStickReset = false;
                    System.out.println("Vision Tracking Stopped Due To Translation Input");
                    stop();
                }
            } else if (getState() != ControlState.MANUAL) {
                setState(ControlState.MANUAL);
            }
        } else if (!Util.epsilonEquals(rotationalInput, 0.0)) {
            if (getState() != ControlState.MANUAL && getState() != ControlState.TRAJECTORY && getState() != ControlState.VISION && getState() != ControlState.POSITION) {
                setState(ControlState.MANUAL);
            }
        }

        if (inputMagnitude > 0.1)
            lastDriveVector = new Translation2d(x, y);
        else if (translationVector.getX() == 0.0 && translationVector.getY() == 0.0 && rotate != 0.0)
            lastDriveVector = rotationalVector;

    }

    private void updateControlCycle(double timestamp) {
        double rotationCorrection = headingController.update(robotPose.getRotation().getDegrees());

        switch (getState()) {
            case MANUAL:
                if (translationVector.equals(new Translation2d()) && rotationalInput == 0.0) {
                    if (lastDriveVector.equals(rotationalVector)) {
                        stop();
                    } else {
                        setOpenLoopCoast(inverseKinematics.updateDriveVectors(lastDriveVector, rotationCorrection, robotPose, getIsRobotCentric()));
                    }
                } else {
                    double scaledRotation = rotationCorrection * (1 - translationVector.getNorm() * 0.4) + rotationalInput;
                    if (Settings.kIsUsingCompBot) {
                        setClosedLoopVelocity(inverseKinematics.updateDriveVectors(translationVector, scaledRotation, robotPose, getIsRobotCentric()));
                    } else {
                        setOpenLoop(inverseKinematics.updateDriveVectors(translationVector, scaledRotation, robotPose, getIsRobotCentric()));
                    }
                }
                break;
            case POSITION:
                // Currently not used
                break;
            case ROTATION:
                setOpenLoop(inverseKinematics.updateDriveVectors(new Translation2d(), Util.deadBand(rotationCorrection, 0.1), robotPose, false));
                break;
            case TRAJECTORY:
                handleTrajectoryUpdate(timestamp, robotPose, rotationCorrection);
                break;
            case VISION:
                handleVisionUpdate(timestamp, robotPose, rotationCorrection);
                break;
            case NEUTRAL:
                stop();
                break;
            default:
                break;
        }
    }

    public void handleTrajectoryUpdate(double timestamp, Pose2d robotPose, double rotationCorrection) {
        if (motionPlanner.isDone()) {
            Translation2d driveVector = motionPlanner.update(timestamp, );
        }
    }

    private void zeroSensorsForAuto() {
        if (!hasStartedTrjectory) {
            if (moduleConfigRequested) {
                zeroSensors();
                System.out.println("Position reset for auto");
            }
            hasStartedTrjectory = true;
        }
    }

    private double calculateTrajectoryRotationalInput(double rotationCorrection, double driveVecNorm) {
        return Util.deadBand(Util.limit(rotationCorrection * rotationScalar * driveVecNorm, motionPlanner.getMaxRotationSpeed()), 0.01);
    }

    private Translation2d handleDriveVecNorm(Translation2d driveVec, double rotationalInput, Pose2d pose) {
        if (Util.epsilonEquals(driveVec.getNorm(), 0.0)) {
            driveVec = lastTrajectoryVector;
            setClosedLoopVelocityStall(inverseKinematics.updateDriveVectors(driveVec, rotationalInput, pose, false));
        } else {
            setClosedLoopVelocity(inverseKinematics.updateDriveVectors(driveVec, rotationalInput, pose, false));
        }
        return driveVec;
    }

    private void updateModuleReadyStatus() {
        if (areModuleAnglesOnTarget() && !modulesReady) {
            modulesReady = true;
            System.out.println("Modules Ready");
        }
    }

    private void logPathCompletion(double timestamp) {
        System.out.println("Path completed in: " + (timestamp - trajectoryStartTime));
        hasFinishedTrajectory = true;
        if (alwaysConfigModules) requireModuleConfig();
    }

    public void handleVisionUpdate(double timestamp, Pose2d robotPose, double rotationCorrection) {

    }

    private final Loop loop = new Loop() {
        @Override
        public void onStart(double timestamp) {

        }

        @Override
        public void onLoop(double timestamp) {

        }

        @Override
        public void onStop(double timestamp) {

        }
    };

    private void rotateInPlace(Rotation2d targetHeading) {
        setState(ControlState.ROTATION);
        headingController.setStationarySnapTarget(targetHeading.getDegrees());
    }

    public void rotate(Rotation2d targetHeading) {
        if (DriverStation.getAlliance().get() != DriverStation.Alliance.Blue) {
            targetHeading = targetHeading.rotateBy(Rotation2d.fromDegrees(180.0));
        }

        if (translationVector.getX() == 0.0 && translationVector.getY() == 0.0) {
            rotateInPlace(targetHeading);
        } else {
            headingController.setMaintainTarget(targetHeading.getDegrees());
        }
    }

    public void setPathHeading(Rotation2d targetHeading) {
        headingController.setMoveSnapTarget(targetHeading.getDegrees());
    }

    public void setOpenLoop(List<Translation2d> driveVectors) {
        modules.forEach(m -> m.setOpenLoop(driveVectors.get(modules.indexOf(m))));
    }

    public void setOpenLoopCoast(List<Translation2d> driveVectors) {
        modules.forEach(m -> m.setOpenLoopCoast(driveVectors.get(modules.indexOf(m)).getAngle()));
    }

    public void setClosedLoopVelocity(List<Translation2d> driveVectors) {
        modules.forEach(m -> m.setClosedLoopVelocity(Translation2dHelper.scale(driveVectors.get(modules.indexOf(m)), Constants.SwerveConfig.kMaxLinearVelocity)));
    }

    public void setClosedLoopVelocityStall(List<Translation2d> driveVectors) {
        modules.forEach(m -> m.setClosedLoopVelocityStall(driveVectors.get(modules.indexOf(m)).getAngle()));
    }

    public boolean areModuleAnglesOnTarget() {
        return modules.stream().allMatch(SwerveModule::isAngleOnTarget);
    }

    public void updateOdometry(double timestamp) {
        robotPose = poseEstimator.updateWithTime(timestamp, Rotation2d.fromDegrees(inputs.heading.getDegrees()), getModulePositions());
    }

    public void zeroModuleAngles() {
        modules.forEach(SwerveModule::resetAzmithToAbsolute);
    }

    public void forceZeroModuleAngles() {
        modules.forEach(SwerveModule::forceResetAzmithToAbsolute);
    }

    public Rotation2d getHeading() {
        return robotPose.getRotation();
    }

    @Override
    public void readPeriodicInputs() {
        inputs.timestamp = Timer.getFPGATimestamp();
        modules.forEach(SwerveModule::readPeriodicInputs);
        inputs.heading = Rotation2d.fromDegrees(gyro.getYaw());
        if (!isPoseEstimatorInitialized) {
            zeroSensorsBasedOnAlliance();
        }
        updateOdometry(inputs.timestamp);
    }

    @Override
    public void writePeriodicOutputs() {
        modules.forEach(SwerveModule::writePeriodicOutputs);
    }

    @Override
    public void outputTelemetry() {
        modules.forEach(SwerveModule::outputTelemetry);
        Logger.recordOutput("Swerve/Robot Pose", getRobotPose());
        Logger.recordOutput("Swerve/Robot Heading", getHeading());
        Logger.recordOutput("Swerve/State", getState());
    }

    @Override
    public void stop() {
        setState(ControlState.NEUTRAL);
        modules.forEach(SwerveModule::stop);
    }

    @Override
    public void zeroSensors() {
        poseEstimator.resetPosition(inputs.heading, getModulePositions(), new Pose2d());
        distanceTraveled = 0.0;
    }

    public void zeroSensorsBasedOnAlliance() {
        double heading = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 0.0 : 180.0;
        gyro.setYaw(heading);
        isPoseEstimatorInitialized = true;
    }

    @Override
    public void registerLooper(ILooper looper) {
        looper.register(loop);
    }

    private static class inputs {
        public static double timestamp;
        public static ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();
        public static Twist2d measuredVelocity = new Twist2d();
        public static Rotation2d heading = new Rotation2d();
    }

    private static class outputs {
        public SwerveModuleState[] moduleStates = new SwerveModuleState[] {
                new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
        };

        public Twist2d predictedVelocity = new Twist2d();
        public Translation2d translationalError = new Translation2d();
        public Rotation2d rotationError = new Rotation2d();
        public Rotation2d rotationSetpoint = new Rotation2d();
    }
}
