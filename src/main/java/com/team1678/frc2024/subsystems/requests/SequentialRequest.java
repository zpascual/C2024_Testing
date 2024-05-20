package com.team1678.frc2024.subsystems.requests;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * A Request which itself takes a list of Requests and executes them in series.
 */
public class SequentialRequest extends Request {
    private final List<Request> requests;
    private Request currentRequest = null;
    private boolean startedCurrentRequest = false;

    public SequentialRequest(Request... requests) {
        this(Arrays.asList(requests));
    }

    public SequentialRequest(List<Request> requests) {
        this.requests = new ArrayList<>(requests);
    }

    @Override
    public void cleanup() {
        if (currentRequest != null) {
            currentRequest.cleanup();
        }
        requests.forEach(Request::cleanup);
        super.cleanup();
    }

    private void startRequestIfAllowed() {
        if (currentRequest.allowed()) {
            currentRequest.act();
            startedCurrentRequest = true;
        }
    }

    @Override
    public void act() {
        currentRequest = requests.remove(0);
        startRequestIfAllowed();
    }

    @Override
    public boolean isFinished(){
        if (!startedCurrentRequest && currentRequest != null) {
            startRequestIfAllowed();
        }

        if (startedCurrentRequest && currentRequest.isFinished()) {
            if (requests.isEmpty()) {
                currentRequest = null;
                return true;
            } 

            currentRequest = requests.remove(0);
            startedCurrentRequest = false;
            startRequestIfAllowed();
        }

        return false;
    }

    @Override
    public String toString() {
        return String.format("SequentialRequest(currentRequest = %s, startedCurrentRequest = %b, remainingRequests = %s)",
                currentRequest, startedCurrentRequest, requests);
    }
}
