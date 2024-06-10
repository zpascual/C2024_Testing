package com.team1678.frc2024.planners;

import com.team1678.frc2024.requests.EmptyRequest;
import com.team1678.frc2024.requests.Request;

public class RequestExecuter {
    private Request activeRequest = null;
    private boolean startedCurrentReqeust = false;

    public void request(Request r) {
        if (activeRequest != null) {
            activeRequest.cleanup();
        }
        activeRequest = r;
        startedCurrentReqeust = false;
    }

    public void update() {
        if (activeRequest == null) {
            return;
        }

        if (!startedCurrentReqeust && activeRequest.allowed()) {
            activeRequest.act();
            startedCurrentReqeust = true;
        }

        if (startedCurrentReqeust && activeRequest.isFinished()) {
            activeRequest = null;
        }
    }

    public boolean isFinished() {return activeRequest == null;}

    public void clear() {request(new EmptyRequest());}

}
