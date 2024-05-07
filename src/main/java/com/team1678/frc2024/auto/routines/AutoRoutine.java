package com.team1678.frc2024.auto.routines;

import com.choreo.lib.*;
import com.team1678.frc2024.requests.LambdaRequest;
import com.team1678.frc2024.requests.Request;
import com.team1678.lib.util.Stopwatch;

import java.util.ArrayList;

public abstract class AutoRoutine implements IAutoRoutine {
    protected final Stopwatch runtimeStopwatch = new Stopwatch();

    protected Request getStartStopwatchRequest() {return runtimeStopwatch.getStartRequest();}

    protected Request getPrintRuntimeRequest() {
        return new LambdaRequest(() -> System.out.printf("Auto finished in %.2f seconds.%n", runtimeStopwatch.getTime()));
    }
}
