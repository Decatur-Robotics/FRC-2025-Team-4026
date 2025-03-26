package frc.robot.constants;

import frc.robot.util.SuperstructureState;

public class SuperstructureConstants {

    // Intaking states

    public static final SuperstructureState CORAL_GROUND_INTAKING_STATE = new SuperstructureState(
        ElevatorConstants.CORAL_GROUND_INTAKING_POSITION, 
        ArmConstants.CORAL_GROUND_INTAKING_POSITION, 
        WristConstants.PERPENDICULAR_CURRENT, 
        IntakeConstants.CORAL_INTAKE_VELOCITY);

    public static final SuperstructureState CORAL_HUMAN_PLAYER_INTAKING_STATE = new SuperstructureState(
        ElevatorConstants.CORAL_HUMAN_PLAYER_INTAKING_POSITION, 
        ArmConstants.CORAL_HUMAN_PLAYER_INTAKING_POSITION, 
        WristConstants.PERPENDICULAR_CURRENT, 
        IntakeConstants.CORAL_INTAKE_VELOCITY);

    public static final SuperstructureState ALGAE_GROUND_INTAKING_STATE = new SuperstructureState(
        ElevatorConstants.ALGAE_GROUND_INTAKING_POSITION, 
        ArmConstants.ALGAE_GROUND_INTAKING_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        IntakeConstants.ALGAE_INTAKE_VELOCITY);

    public static final SuperstructureState ALGAE_LOW_REEF_INTAKING_STATE = new SuperstructureState(
        ElevatorConstants.ALGAE_LOW_REEF_INTAKING_POSITION, 
        ArmConstants.ALGAE_LOW_REEF_INTAKING_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        IntakeConstants.ALGAE_INTAKE_VELOCITY);

    public static final SuperstructureState ALGAE_HIGH_REEF_INTAKING_STATE = new SuperstructureState(
        ElevatorConstants.ALGAE_HIGH_REEF_INTAKING_POSITION, 
        ArmConstants.ALGAE_HIGH_REEF_INTAKING_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        IntakeConstants.ALGAE_INTAKE_VELOCITY);

    // Stowed states

    public static final SuperstructureState CORAL_STOWED_STATE = new SuperstructureState(
        ElevatorConstants.STOWED_POSITION, 
        ArmConstants.STOWED_POSITION, 
        WristConstants.PERPENDICULAR_CURRENT, 
        IntakeConstants.CORAL_REST_VELOCITY);

    public static final SuperstructureState ALGAE_STOWED_STATE = new SuperstructureState(
        ElevatorConstants.STOWED_POSITION, 
        ArmConstants.STOWED_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        IntakeConstants.ALGAE_REST_VELOCITY);

    // L1 scoring states

    public static final SuperstructureState STAGE_L1_STATE = new SuperstructureState(
        ElevatorConstants.L1_POSITION, 
        ArmConstants.L1_SCORING_POSITION, 
        WristConstants.PERPENDICULAR_CURRENT, 
        IntakeConstants.CORAL_REST_VELOCITY);

    public static final SuperstructureState EJECT_L1_STATE = new SuperstructureState(
        ElevatorConstants.L1_POSITION, 
        ArmConstants.L1_SCORING_POSITION, 
        WristConstants.PERPENDICULAR_CURRENT, 
        IntakeConstants.L1_EJECT_VELOCITY);

    // L2 scoring states

    public static final SuperstructureState TRAVEL_L2_STATE = new SuperstructureState(
        ElevatorConstants.STAGE_L2_POSITION, 
        ArmConstants.STOWED_POSITION, 
        WristConstants.PERPENDICULAR_CURRENT, 
        IntakeConstants.CORAL_REST_VELOCITY);

    public static final SuperstructureState STAGE_L2_STATE = new SuperstructureState(
        ElevatorConstants.STAGE_L2_POSITION, 
        ArmConstants.L2_STAGING_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        IntakeConstants.CORAL_REST_VELOCITY);

    public static final SuperstructureState PLACE_L2_STATE = new SuperstructureState(
        ElevatorConstants.SCORE_L2_POSITION, 
        ArmConstants.L2_SCORING_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        IntakeConstants.CORAL_REST_VELOCITY);

    public static final SuperstructureState RETRACT_L2_STATE = new SuperstructureState(
        ElevatorConstants.RETRACT_L2_POSITION, 
        ArmConstants.STOWED_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        IntakeConstants.BRANCH_EJECT_VELOCITY);

    // L3 scoring states

    public static final SuperstructureState TRAVEL_L3_STATE = new SuperstructureState(
        ElevatorConstants.STAGE_L3_POSITION, 
        ArmConstants.STOWED_POSITION, 
        WristConstants.PERPENDICULAR_CURRENT, 
        IntakeConstants.CORAL_REST_VELOCITY);

    public static final SuperstructureState STAGE_L3_STATE = new SuperstructureState(
        ElevatorConstants.STAGE_L3_POSITION, 
        ArmConstants.L3_STAGING_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        IntakeConstants.CORAL_REST_VELOCITY);

    public static final SuperstructureState PLACE_L3_STATE = new SuperstructureState(
        ElevatorConstants.SCORE_L3_POSITION, 
        ArmConstants.L3_SCORING_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        IntakeConstants.CORAL_REST_VELOCITY);

    public static final SuperstructureState RETRACT_L3_STATE = new SuperstructureState(
        ElevatorConstants.RETRACT_L3_POSITION, 
        ArmConstants.STOWED_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        IntakeConstants.BRANCH_EJECT_VELOCITY);

    // L4 scoring states

    public static final SuperstructureState TRAVEL_L4_STATE = new SuperstructureState(
        ElevatorConstants.STAGE_L4_POSITION, 
        ArmConstants.STOWED_POSITION, 
        WristConstants.PERPENDICULAR_CURRENT, 
        IntakeConstants.CORAL_REST_VELOCITY);

    public static final SuperstructureState STAGE_L4_STATE = new SuperstructureState(
        ElevatorConstants.STAGE_L4_POSITION, 
        ArmConstants.L4_STAGING_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        IntakeConstants.CORAL_REST_VELOCITY);

    public static final SuperstructureState PLACE_L4_STATE = new SuperstructureState(
        ElevatorConstants.SCORE_L4_POSITION, 
        ArmConstants.L4_SCORING_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        IntakeConstants.CORAL_REST_VELOCITY);

    public static final SuperstructureState RETRACT_L4_STATE = new SuperstructureState(
        ElevatorConstants.RETRACT_L4_POSITION, 
        ArmConstants.STOWED_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        IntakeConstants.BRANCH_EJECT_VELOCITY);

    // Processor scoring states

    public static final SuperstructureState STAGE_PROCESSOR_STATE = new SuperstructureState(
        ElevatorConstants.PROCESSOR_POSITION, 
        ArmConstants.PROCESSOR_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        IntakeConstants.CORAL_REST_VELOCITY);

    public static final SuperstructureState EJECT_PROCESSOR_STATE = new SuperstructureState(
        ElevatorConstants.PROCESSOR_POSITION, 
        ArmConstants.PROCESSOR_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        IntakeConstants.PROCESSOR_EJECT_VELOCITY);

    // Net scoring states

    public static final SuperstructureState STAGE_NET_STATE = new SuperstructureState(
        ElevatorConstants.NET_POSITION, 
        ArmConstants.NET_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        IntakeConstants.CORAL_REST_VELOCITY);

    public static final SuperstructureState EJECT_NET_STATE = new SuperstructureState(
        ElevatorConstants.NET_POSITION, 
        ArmConstants.NET_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        IntakeConstants.NET_EJECT_VELOCITY);

    // Error margins
    public static final double ELEVATOR_ERROR_MARGIN = 1;
    public static final double ARM_ERROR_MARGIN = 1;
    public static final double WRIST_ERROR_MARGIN = 0;
    public static final double CLAW_ERROR_MARGIN = 0;

    // Stowability
    public static final double ARM_MINIMUM_STOWED_POSITION = 0;
    public static final double ELEVATOR_MINIMUM_UNSTOWED_POSITION = 0;
            
}
