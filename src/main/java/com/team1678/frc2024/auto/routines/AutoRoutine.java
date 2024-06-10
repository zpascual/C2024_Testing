package com.team1678.frc2024.auto.routines;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.team1678.frc2024.requests.LambdaRequest;
import com.team1678.frc2024.requests.Request;
import com.team1678.lib.util.Stopwatch;

import java.util.List;

public abstract class AutoRoutine implements IAutoRoutine {
    protected ChoreoTrajectory trajectory = null;
    protected final Stopwatch runtimeStopwatch = new Stopwatch();

    public AutoRoutine() {}

    public AutoRoutine(ChoreoTrajectory trajectory) {
        this.trajectory = trajectory;
    }

    public AutoRoutine(ChoreoTrajectory trajectory, boolean isRed) {
        this.trajectory = isRed ? trajectory.flipped() : trajectory;
    }

    public abstract Request getRoutine();

    public List<ChoreoTrajectoryState> getPaths() {return trajectory.getSamples();}

    protected Request getStartStopwatchRequest() {return runtimeStopwatch.getStartRequest();}

    protected Request getPrintRuntimeRequest() {
        return new LambdaRequest(() -> System.out.printf("Auto finished in %.2f seconds.%n", runtimeStopwatch.getTime()));
    }
}
