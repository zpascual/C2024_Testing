package com.team1678.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.team1678.frc2024.requests.Request;

import java.util.ArrayList;

public class TestAuto extends AutoRoutine {
    protected ArrayList<ChoreoTrajectory> trajectories;

    public TestAuto() {
        this.trajectories = Choreo.getTrajectoryGroup("test");
    }

    @Override
    public Request getRoutine() {
        return null;
    }

}
