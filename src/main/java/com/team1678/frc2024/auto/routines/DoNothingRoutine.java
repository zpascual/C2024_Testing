package com.team1678.frc2024.auto.routines;

import com.team1678.frc2024.requests.LambdaRequest;
import com.team1678.frc2024.requests.Request;

public class DoNothingRoutine extends AutoRoutine {
    @Override
    public Request getRoutine() {
        return new LambdaRequest(() -> System.out.println("Starting stand still routine...Done!"));
    }
}
