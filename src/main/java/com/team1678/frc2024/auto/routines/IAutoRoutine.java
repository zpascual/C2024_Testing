package com.team1678.frc2024.auto.routines;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.team1678.frc2024.requests.Request;

import java.util.ArrayList;

public interface IAutoRoutine {
    Request getRoutine();
}
