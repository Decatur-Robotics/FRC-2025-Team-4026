package frc.robot.constants;

import frc.robot.util.SuperstructureState;

public class SuperstructureConstants {

    // Intaking states
    public static final SuperstructureState CORAL_GROUND_INTAKING_STATE = new SuperstructureState(
        ElevatorConstants.CORAL_GROUND_INTAKING_POSITION, 
        ArmConstants.CORAL_GROUND_INTAKING_POSITION, 
        WristConstants.PERPENDICULAR_CURRENT, 
        ClawConstants.CLOSED_CURRENT,
        IntakeConstants.INTAKE_VELOCITY);

    public static final SuperstructureState CORAL_HUMAN_PLAYER_INTAKING_STATE = new SuperstructureState(
        ElevatorConstants.CORAL_HUMAN_PLAYER_INTAKING_POSITION, 
        ArmConstants.CORAL_HUMAN_PLAYER_INTAKING_POSITION, 
        WristConstants.PERPENDICULAR_CURRENT, 
        ClawConstants.CLOSED_CURRENT,
        IntakeConstants.INTAKE_VELOCITY);

    public static final SuperstructureState ALGAE_GROUND_INTAKING_STATE = new SuperstructureState(
        ElevatorConstants.ALGAE_GROUND_INTAKING_POSITION, 
        ArmConstants.ALGAE_GROUND_INTAKING_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        ClawConstants.OPEN_CURRENT,
        IntakeConstants.INTAKE_VELOCITY);

    public static final SuperstructureState ALGAE_LOW_REEF_INTAKING_STATE = new SuperstructureState(
        ElevatorConstants.ALGAE_LOW_REEF_INTAKING_POSITION, 
        ArmConstants.ALGAE_LOW_REEF_INTAKING_POSITION, 
        WristConstants.PERPENDICULAR_CURRENT, 
        ClawConstants.OPEN_CURRENT,
        IntakeConstants.INTAKE_VELOCITY);

    public static final SuperstructureState ALGAE_HIGH_REEF_INTAKING_STATE = new SuperstructureState(
        ElevatorConstants.ALGAE_HIGH_REEF_INTAKING_POSITION, 
        ArmConstants.ALGAE_HIGH_REEF_INTAKING_POSITION, 
        WristConstants.PERPENDICULAR_CURRENT, 
        ClawConstants.OPEN_CURRENT,
        IntakeConstants.INTAKE_VELOCITY);

    // Stowed states
    public static final SuperstructureState CORAL_STOWED_STATE = new SuperstructureState(
        ElevatorConstants.STOWED_POSITION, 
        ArmConstants.STOWED_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        ClawConstants.CLOSED_CURRENT,
        IntakeConstants.REST_VELOCITY);

    public static final SuperstructureState ALGAE_STOWED_STATE = new SuperstructureState(
        ElevatorConstants.STOWED_POSITION, 
        ArmConstants.STOWED_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        ClawConstants.CLOSED_CURRENT,
        IntakeConstants.REST_VELOCITY);

    // Scoring states
    public static final SuperstructureState MOVE_TO_L1_STATE = new SuperstructureState(
        ElevatorConstants.L1_POSITION, 
        ArmConstants.L1_POSITION, 
        WristConstants.PERPENDICULAR_CURRENT, 
        ClawConstants.CLOSED_CURRENT,
        IntakeConstants.REST_VELOCITY);

    public static final SuperstructureState MOVE_TO_L2_STATE = new SuperstructureState(
        ElevatorConstants.MOVE_TO_L2_POSITION, 
        ArmConstants.MOVE_TO_L2_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        ClawConstants.CLOSED_CURRENT,
        IntakeConstants.REST_VELOCITY);

    public static final SuperstructureState MOVE_TO_L3_STATE = new SuperstructureState(
        ElevatorConstants.MOVE_TO_L3_POSITION, 
        ArmConstants.MOVE_TO_L3_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        ClawConstants.CLOSED_CURRENT,
        IntakeConstants.REST_VELOCITY);

    public static final SuperstructureState MOVE_TO_L4_STATE = new SuperstructureState(
        ElevatorConstants.MOVE_TO_L4_POSITION, 
        ArmConstants.MOVE_TO_L4_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        ClawConstants.CLOSED_CURRENT,
        IntakeConstants.REST_VELOCITY);

    public static final SuperstructureState MOVE_TO_PROCESSOR_STATE = new SuperstructureState(
        ElevatorConstants.PROCESSOR_POSITION, 
        ArmConstants.PROCESSOR_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        ClawConstants.OPEN_CURRENT,
        IntakeConstants.REST_VELOCITY);

    public static final SuperstructureState MOVE_TO_NET_STATE = new SuperstructureState(
        ElevatorConstants.NET_POSITION, 
        ArmConstants.NET_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        ClawConstants.OPEN_CURRENT,
        IntakeConstants.REST_VELOCITY);

    public static final SuperstructureState SCORE_L1_STATE = new SuperstructureState(
        ElevatorConstants.L1_POSITION, 
        ArmConstants.L1_POSITION, 
        WristConstants.PERPENDICULAR_CURRENT, 
        ClawConstants.CLOSED_CURRENT,
        IntakeConstants.L1_EJECT_VELOCITY);

    public static final SuperstructureState SCORE_L2_STATE = new SuperstructureState(
        ElevatorConstants.SCORE_L2_POSITION, 
        ArmConstants.SCORE_L2_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        ClawConstants.OPEN_CURRENT,
        IntakeConstants.REST_VELOCITY);

    public static final SuperstructureState SCORE_L3_STATE = new SuperstructureState(
        ElevatorConstants.SCORE_L3_POSITION, 
        ArmConstants.SCORE_L3_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        ClawConstants.OPEN_CURRENT,
        IntakeConstants.REST_VELOCITY);

    public static final SuperstructureState SCORE_L4_STATE = new SuperstructureState(
        ElevatorConstants.SCORE_L4_POSITION, 
        ArmConstants.SCORE_L4_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        ClawConstants.OPEN_CURRENT,
        IntakeConstants.REST_VELOCITY);

    public static final SuperstructureState SCORE_PROCESSOR_STATE = new SuperstructureState(
        ElevatorConstants.PROCESSOR_POSITION, 
        ArmConstants.PROCESSOR_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        ClawConstants.OPEN_CURRENT,
        IntakeConstants.PROCESSOR_EJECT_VELOCITY);

    public static final SuperstructureState SCORE_NET_STATE = new SuperstructureState(
        ElevatorConstants.NET_POSITION, 
        ArmConstants.NET_POSITION, 
        WristConstants.PARALLEL_CURRENT, 
        ClawConstants.OPEN_CURRENT,
        IntakeConstants.NET_EJECT_VELOCITY);

    // Error margins
    public static final double ELEVATOR_ERROR_MARGIN = 0;
    public static final double ARM_ERROR_MARGIN = 0;
    public static final double WRIST_ERROR_MARGIN = 0;
    public static final double CLAW_ERROR_MARGIN = 0;

    // Stowability
    public static final double ARM_MINIMUM_STOWED_POSITION = 0;
    public static final double ELEVATOR_MINIMUM_UNSTOWED_POSITION = 0;

    /** Seconds to wait before stowing superstructure after scoring coral */
    public static final double CORAL_SCORE_TO_STOW_DELAY = 0.25;
    /** Seconds to wait before stowing superstructure after scoring algae */
    public static final double ALGAE_SCORE_TO_STOW_DELAY = 0.25;
            
}
