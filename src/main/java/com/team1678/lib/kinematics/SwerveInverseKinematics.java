package com.team1678.lib.kinematics;

import com.team1678.frc2024.Constants;
import com.team1678.lib.util.Rotation2dHelper;
import com.team1678.lib.util.Translation2dHelper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class SwerveInverseKinematics {
    private List<Translation2d> moduleRelativePositions;
    private List<Translation2d> moduleRotationDirections = updateRotationDirections();

    public SwerveInverseKinematics(List<Translation2d> modulePositions) {
        this.moduleRelativePositions = modulePositions;
    }

    private List<Translation2d> updateRotationDirections() {
        assert moduleRelativePositions != null;
        return moduleRelativePositions.stream().map(moduleRelativePosition -> moduleRelativePosition.rotateBy(Rotation2d.fromDegrees(90)))
                .collect(Collectors.toList());
    }

    public void setCenterOfRotation(Translation2d center) {
        List<Translation2d> positions = Constants.SwerveModules.kModulePositions.stream()
                .map(p -> p.minus(center))
                .collect(Collectors.toList());

        double maxMagnitude = positions.stream()
                .mapToDouble(Translation2d::getNorm)
                .max()
                .orElse(1.0);

        positions.replaceAll(p -> Translation2dHelper.scale(p, 1.0 / maxMagnitude));

        moduleRelativePositions = positions;
        moduleRotationDirections = updateRotationDirections();
    }

    public List<Translation2d> updateDriveVectors(Translation2d transVec, double rotMag, Pose2d robotPose, boolean robotCentric) {
        if (!robotCentric) transVec = transVec.rotateBy(Rotation2dHelper.inverse(robotPose.getRotation()));

        Translation2d finalTransVec = transVec;
        List<Translation2d> driveVec = IntStream.range(0, moduleRelativePositions.size())
                .mapToObj(i -> finalTransVec.plus(Translation2dHelper.scale(moduleRotationDirections.get(i), rotMag)))
                .toList();

        double maxMagnitude = driveVec.stream()
                .mapToDouble(Translation2d::getNorm)
                .max()
                .orElse(1.0);

        return driveVec.stream()
                .map(v -> Translation2dHelper.scale(v, 1.0/maxMagnitude))
                .collect(Collectors.toList());
    }

}
