package frc.robot.constants;

import frc.robot.util.SuperstructureState;

public class SuperstructureConstants {

    // Intaking states
    public static final SuperstructureState CORAL_GROUND_INTAKING_STATE = new SuperstructureState(
            0, 
            0, 
            0, 
            0);

    public static final SuperstructureState CORAL_HUMAN_PLAYER_INTAKING_STATE = new SuperstructureState(
            0, 
            0, 
            0, 
            0);

    public static final SuperstructureState ALGAE_GROUND_INTAKING_STATE = new SuperstructureState(
            0, 
            0, 
            0, 
            0);

    public static final SuperstructureState ALGAE_LOW_REEF_INTAKING_STATE = new SuperstructureState(
            0, 
            0, 
            0, 
            0);

    public static final SuperstructureState ALGAE_HIGH_REEF_INTAKING_STATE = new SuperstructureState(
            0, 
            0, 
            0, 
            0);

    // Stowed states
    public static final SuperstructureState CORAL_STOWED_STATE = new SuperstructureState(
            0, 
            0, 
            0, 
            0);

    public static final SuperstructureState ALGAE_STOWED_STATE = new SuperstructureState(
            0, 
            0, 
            0, 
            0);

    // Scoring states
    public static final SuperstructureState L1_SCORING_STATE = new SuperstructureState(
            0, 
            0, 
            0, 
            0);

    public static final SuperstructureState L2_SCORING_STATE = new SuperstructureState(
            0, 
            0, 
            0, 
            0);

    public static final SuperstructureState L3_SCORING_STATE = new SuperstructureState(
            0, 
            0, 
            0, 
            0);

    public static final SuperstructureState L4_SCORING_STATE = new SuperstructureState(
            0, 
            0, 
            0, 
            0);

    public static final SuperstructureState PROCESSOR_SCORING_STATE = new SuperstructureState(
            0, 
            0, 
            0, 
            0);

    public static final SuperstructureState NET_SCORING_STATE = new SuperstructureState(
            0, 
            0, 
            0, 
            0);

    // Error margins
    public static final double ELEVATOR_ERROR_MARGIN = 0;
    public static final double ARM_ERROR_MARGIN = 0;
    public static final double WRIST_ERROR_MARGIN = 0;
    public static final double CLAW_ERROR_MARGIN = 0;

    // Stowability
    public static final double ARM_MINIMUM_STOWED_POSITION = 0;
    public static final double ELEVATOR_MINIMUM_UNSTOWED_POSITION = 0;
            
}
