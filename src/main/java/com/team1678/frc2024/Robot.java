// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1678.frc2024;

import com.team1678.frc2024.field.AllianceChooser;
import com.team1678.frc2024.auto.AutoModeSelector;
import com.team1678.frc2024.loops.AutoLoop;
import com.team1678.frc2024.loops.SynchronousLooper;
import com.team1678.frc2024.subsystems.SubsystemManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

    private SubsystemManager subsystemManager;

    private AutoModeSelector autoModeSelector = new AutoModeSelector();
    private AutoLoop autoLoop;

    private SynchronousLooper looper;
    private DriverControls driverControls;

    public Robot() {
        super(Constants.kMainThreadDt);
    }

    @Override
    public void robotInit() {
        RobotController.setBrownoutVoltage(6.1);

        if (RobotBase.isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("/media/sda1"));
            Logger.addDataReceiver(new NT4Publisher());
        } else if (Settings.kRobotMode == Settings.RobotMode.SIM) {
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        } else {
            Logger.addDataReceiver(new WPILOGWriter(""));
            Logger.addDataReceiver(new NT4Publisher());
        }

        Logger.start();

        autoLoop = new AutoLoop(autoModeSelector);
        driverControls = DriverControls.getInstance();
        subsystemManager = driverControls.getSubsystems();
        looper = new SynchronousLooper(subsystemManager);

        looper.registerAutoLoop(autoLoop);
        looper.registerTeleopLoop(driverControls);
        looper.registerTeleopLoop(subsystemManager);

        autoModeSelector.initWithDefaults();
    }
    
    
    @Override
    public void robotPeriodic() {
        Logger.recordOutput("Enabled", DriverStation.isEnabled());
        Logger.recordOutput("Match Time", DriverStation.getMatchTime());
        Logger.recordOutput("Battery Voltage", RobotController.getBatteryVoltage());
    }
    
    
    @Override
    public void autonomousInit() {
        AllianceChooser.update();
        looper.startAuto(Timer.getFPGATimestamp());
    }
    
    
    @Override
    public void autonomousPeriodic() {
        looper.onAutoLoop(Timer.getFPGATimestamp());
    }
    
    
    @Override
    public void teleopInit() {
        AllianceChooser.update();
        looper.startTeleop(Timer.getFPGATimestamp());
    }
    
    
    @Override
    public void teleopPeriodic() {
        looper.onTeleopLoop(Timer.getFPGATimestamp());
    }
    
    
    @Override
    public void disabledInit() {
        AllianceChooser.update();
        looper.startDisabled(Timer.getFPGATimestamp());
    }
    
    
    @Override
    public void disabledPeriodic() {
        autoModeSelector.output();
        looper.onDisabledLoop(Timer.getFPGATimestamp());
    }
    
    
    @Override
    public void simulationInit() {}
    
    
    @Override
    public void simulationPeriodic() {}
}
