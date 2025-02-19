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

    /** The encoder value when the arm is straight up */
    public static final int THROUGH_BORE_ENCODER_ZERO_OFFSET = 0;
    public static final int THROUGH_BORE_ENCODER_TO_TALON_ENCODER_RATIO = 8192; // how many encoder ticks it takes to complete one rotation of the mechanism for this specific encoder
    
}
