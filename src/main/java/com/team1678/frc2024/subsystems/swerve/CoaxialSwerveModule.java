package com.team1678.frc2024.subsystems.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1678.frc2024.Constants;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.lib.drivers.motors.MotorController;
import com.team1678.lib.drivers.motors.Phoenix6FXMotorController;
import com.team1678.lib.util.LogUtil;
import com.team1678.lib.util.Util;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class CoaxialSwerveModule extends SwerveModule {

    private final MotorController azmithMotor, driveMotor;
    private final CoaxialSwerveInputs inputs = new CoaxialSwerveInputs();
    private final CoaxialSwerveOutputs outputs = new CoaxialSwerveOutputs();
    private final ControlModeValue azmithClosedLoopControlMode = ControlModeValue.MotionMagicDutyCycleFOC;
    private final ControlModeValue azmithOpenLoopControlMode = ControlModeValue.DutyCycleFOC;
    private final ControlModeValue driveOpenLoopControlMode = ControlModeValue.DutyCycleFOC;
    private final ControlModeValue driveVelocityControlMode = ControlModeValue.VelocityDutyCycleFOC;
    private final ControlModeValue drivePositionControlMode = ControlModeValue.MotionMagicDutyCycleFOC;

    public CoaxialSwerveModule(int moduleId, double encoderOffsetDegrees, boolean flipAbsoluteEncoder, EncoderType encoderType, SwerveMotorInfo azmithMotorInfo, SwerveMotorInfo driveMotorInfo) {
        super(moduleId, encoderOffsetDegrees, flipAbsoluteEncoder, encoderType);
        azmithMotor = createAzmithMotor(azmithMotorInfo);
        driveMotor = createDriveMotor(driveMotorInfo);
    }

    private MotorController createDriveMotor(SwerveMotorInfo motorInfo) {
        MotorController motor = Phoenix6FXMotorController.createRealOrSimulatedController(motorInfo.deviceId, motorInfo.bus, true);
        motor.configureAsCoaxialSwerveDrive();
        motor.setInverted(motorInfo.invertType);
        return motor;
    }

    private MotorController createAzmithMotor(SwerveMotorInfo motorInfo) {
        MotorController motor = Phoenix6FXMotorController.createRealOrSimulatedController(motorInfo.deviceId, motorInfo.bus, true);
        motor.configureAsCoaxialSwerveRotation();
        motor.setInverted(motorInfo.invertType);
        return motor;
    }

    @Override
    public void readPeriodicInputs() {
        inputs.azmithPosition = azmithMotor.getInternalRotorPosition();
        inputs.drivePosition = driveMotor.getInternalRotorPosition();
        inputs.driveVelocity = driveMotor.getInternalRotorVelocity();
        inputs.absoluteAzmithPosition = azmithAbsoluteEncoder.getDegrees();
        Logger.processInputs(moduleName, (LoggableInputs) inputs);
    }

    @Override
    public void writePeriodicOutputs() {
        azmithMotor.set(outputs.azmithControlMode, outputs.azmithDemand);
        driveMotor.set(outputs.driveControlMode, outputs.driveDemand);
    }

    @Override
    public void outputTelemetry() {
        LogUtil.recordRotation2dDeg(getLogKey("Angle Deg"), getAngle());
        LogUtil.recordRotation2dRad(getLogKey("Angle Rad"), getAngle());
        Logger.recordOutput("Angle Rotation2d", getAngle());
        Logger.recordOutput("Meters Driven", getDistanceDrivenMeters());
        Logger.recordOutput("Meters per Second", getDriveVelocityMPS());
        Logger.recordOutput("Drive Supply Current", driveMotor.getSupplyAmps());
        Logger.recordOutput("Drive Stator Current", driveMotor.getStatorAmps());
        Logger.recordOutput("Drive Temperature", driveMotor.getMotorTemperature());
        Logger.recordOutput("Drive TalonFx Voltage", driveMotor.getAppliedVoltage());
    }

    @Override
    public void stop() {
        setAngle(getAngle(), azmithClosedLoopControlMode);
        outputs.driveControlMode = driveOpenLoopControlMode;
        outputs.driveDemand = 0.0;
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void registerLooper(ILooper looper) {

    }

    @Override
    public double getAbsoluteEncoderDegrees() {
        return inputs.absoluteAzmithPosition;
    }

    private double rotorPositionToDegrees(double rotorPosition) {
        return rotorPosition / Constants.SwerveConfig.kAzmithRatio * 360.0;
    }

    private double degreesToRotorPosition(double degrees) {
        return degrees / 360.0 * Constants.SwerveConfig.kAzmithRatio;
    }

    private double getUnboundedAngle() {
        return rotorPositionToDegrees(inputs.azmithPosition);
    }

    private void setAngle(Rotation2d angle, ControlModeValue controlModeValue) {
        double desiredAngle = Util.placeInAppropriate0To360Scope(getUnboundedAngle(), angle.getDegrees());
        double setpoint = degreesToRotorPosition(desiredAngle);
        outputs.azmithControlMode = controlModeValue;
        outputs.azmithDemand = setpoint;
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(getUnboundedAngle());
    }

    @Override
    public boolean isAngleOnTarget() {
        if (outputs.azmithControlMode == ControlModeValue.MotionMagicDutyCycle || outputs.azmithControlMode == ControlModeValue.MotionMagicDutyCycleFOC) {
            double errorDegrees = rotorPositionToDegrees(Math.abs(outputs.azmithDemand - inputs.azmithPosition));
            return errorDegrees < Constants.SwerveConfig.kAzmithTolerance;
        }
        return false;
    }

    @Override
    protected StatusCode setAzmithSensorDegrees(double desiredAngle) {
        return azmithMotor.setSelectedSensorPosition(degreesToRotorPosition(desiredAngle));
    }

    private double rotorPositionToMeters(double rotorPosition) {
        return rotorPosition / Constants.SwerveConfig.kDriveRotorRevPerMeter;
    }

    private double rotorPositionToInches(double rotorPosition) {
        return Util.convertMetersToInches(rotorPositionToMeters(rotorPosition));
    }

    private double metersToRotorPosition(double meters) {
        return meters * Constants.SwerveConfig.kDriveRotorRevPerMeter;
    }

    private double inchesToRotorPosition(double inches) {
        return Util.convertInchesToMeters(inches) * Constants.SwerveConfig.kDriveRotorRevPerMeter;
    }

    @Override
    public double getDistanceDrivenMeters() {
        return rotorPositionToMeters(inputs.drivePosition);
    }

    public double getDistanceDrivenInches() {
        return Util.convertMetersToInches(getDistanceDrivenMeters());
    }

    private double rotorVelToMetersPerSecond(double rotorVelocity) {
        return rotorPositionToMeters(rotorVelocity);
    }

    private double metersPerSecondToRotorVel(double metersPerSecond) {
        return metersToRotorPosition(metersPerSecond);
    }

    @Override
    public double getDriveVelocityMPS() {
        return rotorVelToMetersPerSecond(inputs.driveVelocity);
    }

    @Override
    public double getDriveVoltage() {
        return driveMotor.getAppliedVoltage();
    }

    @Override
    public boolean isDrivePositionOnTarget() {
        if (outputs.driveControlMode == ControlModeValue.MotionMagicDutyCycle || outputs.driveControlMode == ControlModeValue.MotionMagicDutyCycleFOC) {
            return rotorPositionToInches(Math.abs(outputs.driveDemand - inputs.drivePosition)) < Constants.SwerveConfig.kMotionMagicDriveToleranceInches;
        }
        return false;
    }

    @Override
    public void setDriveNeutralMode(NeutralModeValue mode) {
        driveMotor.setNeutralModeValue(mode);
    }

    @Override
    public void setAzmithNeutralMode(NeutralModeValue mode) {
        azmithMotor.setNeutralModeValue(mode);
    }

    @Override
    public void setOpenLoop(Translation2d driveVector) {
        Rotation2d driveDirection = driveVector.getAngle();

        if (Util.shouldReverse(driveDirection, getAngle())) {
            setAngle(driveDirection.rotateBy(Rotation2d.fromDegrees(180.0)), azmithClosedLoopControlMode);
            outputs.driveDemand = -driveVector.getNorm();
        } else {
            setAngle(driveDirection, azmithClosedLoopControlMode);
            outputs.driveDemand = driveVector.getNorm();
        }

        outputs.driveControlMode = driveOpenLoopControlMode;
    }

    @Override
    public void setOpenLoopCoast(Rotation2d driveDirection) {
        if (Util.shouldReverse(driveDirection, getAngle())) {
            setAngle(driveDirection.rotateBy(driveDirection.rotateBy(Rotation2d.fromDegrees(180.0))), azmithClosedLoopControlMode);
        } else {
            setAngle(driveDirection, azmithClosedLoopControlMode);
        }

        outputs.driveControlMode = driveOpenLoopControlMode;
        outputs.driveDemand = 0.0;
    }

    @Override
    public void setClosedLoopVelocity(Translation2d driveVector) {
        Rotation2d driveDirection = driveVector.getAngle();

        if (Util.shouldReverse(driveDirection, getAngle())) {
            setAngle(driveDirection.rotateBy(Rotation2d.fromDegrees(180.0)), azmithClosedLoopControlMode);
            outputs.driveDemand = -metersPerSecondToRotorVel(driveVector.getNorm());
        } else {
            setAngle(driveDirection, azmithClosedLoopControlMode);
            outputs.driveDemand = metersPerSecondToRotorVel(driveVector.getNorm());
        }

        driveMotor.selectProfileSlot(1);
        outputs.driveControlMode = driveVelocityControlMode;
    }

    @Override
    public void setClosedLoopVelocityStall(Rotation2d driveDirection) {
        if (Util.shouldReverse(driveDirection, getAngle())) {
            setAngle(driveDirection.rotateBy(Rotation2d.fromDegrees(180.0)), azmithClosedLoopControlMode);
        } else {
            setAngle(driveDirection, azmithClosedLoopControlMode);
        }

        outputs.driveControlMode = driveVelocityControlMode;
        outputs.driveDemand = 0.0;
    }

    @Override
    public void setClosedLoopPosition(Translation2d driveVector) {
        setAngle(driveVector.getAngle(), azmithClosedLoopControlMode);

        driveMotor.selectProfileSlot(0);
        outputs.driveControlMode = drivePositionControlMode;
        outputs.driveDemand = inputs.drivePosition + metersToRotorPosition(driveVector.getNorm());
    }

    @Override
    public void disable() {
        outputs.driveControlMode = driveOpenLoopControlMode;
        outputs.driveDemand = 0.0;

        outputs.azmithControlMode = azmithOpenLoopControlMode;
        outputs.azmithDemand = 0.0;
    }

    public static class CoaxialSwerveInputs {
        public double azmithPosition;
        public double drivePosition;
        public double driveVelocity;
        public double absoluteAzmithPosition;
    }

    public static class CoaxialSwerveOutputs {
        public ControlModeValue azmithControlMode = ControlModeValue.DutyCycleFOC;
        public double azmithDemand = 0.0;
        public ControlModeValue driveControlMode = ControlModeValue.DutyCycleFOC;
        public double driveDemand = 0.0;
    }
}
