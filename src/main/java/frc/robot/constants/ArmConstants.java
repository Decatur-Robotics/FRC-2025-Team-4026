package frc.robot.constants;

public class ArmConstants {

    public static final double KP = 0;
	public static final double KI = 0;
	public static final double KD = 0;
	public static final double KS = 0;
	public static final double KV = 0;
	public static final double KA = 0;
	public static final double KG = 0;
    
    public static final double ACCELERATION = 0;
    public static final double CRUISE_VELOCITY = 0;

    public static final double ENCODER_TO_RADIANS_FACTOR = 0;
    
    public static final double CURRENT_LIMIT = 0;

    /** The position when the arm is parallel to the floor */
	public static final double LEVEL_POSITION = 0;

    public static final double INITIAL_POSITION = SuperstructureConstants.CORAL_STOWED_STATE.armPosition;
    public static final double K_ENCODER_COUNTS_PER_REVOLUTION = 8192; // how many ticks it takes to complete one rotation for this specific encoder
    
}
