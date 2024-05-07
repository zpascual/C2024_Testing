package com.team1678.frc2024.auto.routines;

import com.team1678.frc2024.requests.LambdaRequest;
import com.team1678.frc2024.requests.Request;
import com.team1678.lib.util.Stopwatch;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.TrajectoryGenerator.TrajectorySet;
import com.team254.lib.trajectory.timing.TimedState;

import java.util.ArrayList;
import java.util.List;

public abstract class AutoRoutine {
    protected final TrajectorySet trajectories;
    protected final Stopwatch runtimeStopwatch = new Stopwatch();

    public AutoRoutine() {
        trajectories = TrajectoryGenerator.getInstance().getTrajectorySet();
    }

    public abstract Request getRoutine();

    public List<Trajectory<TimedState<Pose2dWithMotion>>> getPaths() {return new ArrayList<>();}

    protected Request getStartStopwatchRequest() {return runtimeStopwatch.getStartRequest();}

    protected Request getPrintRuntimeRequest() {
        return new LambdaRequest(() -> System.out.printf("Auto finished in %.2f seconds.%n", runtimeStopwatch.getTime()));
    }
}
