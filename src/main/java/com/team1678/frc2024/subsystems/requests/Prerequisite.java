package com.team1678.frc2024.subsystems.requests;

/**
 * A state which must be met before a Request can be acted upon
 */
public interface Prerequisite {
    public abstract boolean met();
}
