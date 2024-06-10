package com.team1678.frc2024.requests;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team1678.frc2024.requests.LambdaRequest.VoidInterface;

public abstract class Request {

	public abstract void act();
	
	public abstract boolean isFinished();

	private final List<Prerequisite> prerequisites = new ArrayList<>();

	public Request withPrerequisites(Prerequisite... prereqs) {
        prerequisites.addAll(Arrays.asList(prereqs));
		return this;
	}

	public Request withPrerequisite(Prerequisite prereq) {
		prerequisites.add(prereq);
		return this;
	}

	public boolean allowed() {
		return prerequisites.stream().allMatch(Prerequisite::met);
	}

	private VoidInterface cleanupFunction = () -> {};

	public Request withCleanup(VoidInterface cleanupFunction) {
		this.cleanupFunction = cleanupFunction;
		return this;
	}

	public void cleanup() {
		cleanupFunction.f();
	}

}
