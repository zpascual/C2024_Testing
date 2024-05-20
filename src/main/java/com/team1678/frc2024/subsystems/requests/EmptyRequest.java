package com.team1678.frc2024.subsystems.requests;

public class EmptyRequest extends Request{
	
	@Override
	public void act(){
		// empty, as the name suggests
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public String toString() {
		return "EmptyRequest()";
	}
	
}
