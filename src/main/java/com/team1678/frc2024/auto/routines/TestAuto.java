package com.team1678.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.team1678.frc2024.subsystems.requests.Request;

import java.util.ArrayList;

public class TestAuto extends AutoRoutine {
    protected ArrayList<ChoreoTrajectory> trajectories;

    public TestAuto() {
        this.trajectories = Choreo.getTrajectoryGroup("test");
        System.out.printf("Number of trajectories found: %s", trajectories.size());
    }

    @Override
    public Request getRoutine() {
        return null;
    }

}
