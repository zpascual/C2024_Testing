package com.team1678.frc2024.requests;

import java.util.Arrays;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

/**
 * A Request which takes a list of Requests and executes them in parallel.
 */
public class ParallelRequest extends Request {
    private final List<Request> idleRequests;
    private final List<Request> inProgressRequests;

    public ParallelRequest(Request... requests) {
        idleRequests = new LinkedList<>(Arrays.asList(requests));
        inProgressRequests = new LinkedList<>();
    }

    @Override
    public void cleanup() {
        inProgressRequests.forEach(Request::cleanup);
        idleRequests.forEach(Request::cleanup);
        super.cleanup();
    }

    private void startRequestsIfAllowed() {
        for (Iterator<Request> iter = idleRequests.iterator(); iter.hasNext();) {
            Request request = iter.next();
            if (request.allowed()) {
                request.act();
                inProgressRequests.add(request);
                iter.remove();
            }
        }
    }

    @Override
    public void act() {
        startRequestsIfAllowed();
    }

    @Override
    public boolean isFinished() {
        startRequestsIfAllowed();
        inProgressRequests.removeIf(Request::isFinished);

        return idleRequests.isEmpty() && inProgressRequests.isEmpty();
    }

    @Override
    public String toString() {
        return String.format("ParallelRequest(inProgressRequests = %s, idleRequests = %s)",
                inProgressRequests, idleRequests);   
    }
}
