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
    public static final SuperstructureState MOVE_TO_L1_STATE = new SuperstructureState(
            0, 
            0, 
            0, 
            0);

    public static final SuperstructureState MOVE_TO_L2_STATE = new SuperstructureState(
            0, 
            0, 
            0, 
            0);

    public static final SuperstructureState MOVE_TO_L3_STATE = new SuperstructureState(
            0, 
            0, 
            0, 
            0);

    public static final SuperstructureState MOVE_TO_L4_STATE = new SuperstructureState(
            0, 
            0, 
            0, 
            0);

    public static final SuperstructureState MOVE_TO_PROCESSOR_STATE = new SuperstructureState(
            0, 
            0, 
            0, 
            0);

    public static final SuperstructureState MOVE_TO_NET_STATE = new SuperstructureState(
            0, 
            0, 
            0, 
            0);

    public static final SuperstructureState SCORE_L1_STATE = new SuperstructureState(
            0, 
            0, 
            0, 
            0);

    public static final SuperstructureState SCORE_L2_STATE = new SuperstructureState(
            0, 
            0, 
            0, 
            0);

    public static final SuperstructureState SCORE_L3_STATE = new SuperstructureState(
            0, 
            0, 
            0, 
            0);

    public static final SuperstructureState SCORE_L4_STATE = new SuperstructureState(
            0, 
            0, 
            0, 
            0);

    public static final SuperstructureState SCORE_PROCESSOR_STATE = new SuperstructureState(
            0, 
            0, 
            0, 
            0);

    public static final SuperstructureState SCORE_NET_STATE = new SuperstructureState(
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
