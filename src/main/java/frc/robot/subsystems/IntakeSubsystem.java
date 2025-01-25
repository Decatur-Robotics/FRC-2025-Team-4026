package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClawConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.Ports;

public class IntakeSubsystem extends SubsystemBase {

    private SparkMax motorLeft, motorRight;
    private SparkClosedLoopController intakeController;

    private double velocity;
    
    public IntakeSubsystem() {
        motorLeft = new SparkMax(Ports.INTAKE_MOTOR_LEFT, MotorType.kBrushless);
        motorRight = new SparkMax(Ports.INTAKE_MOTOR_RIGHT, MotorType.kBrushless);

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.idleMode(IdleMode.kBrake);
        followerConfig.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
        followerConfig.follow(motorLeft.getDeviceId());
 
        SparkMaxConfig mainConfig = new SparkMaxConfig();
        mainConfig.idleMode(IdleMode.kBrake);
        mainConfig.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
        mainConfig.closedLoop.pidf(IntakeConstants.KP, IntakeConstants.KI, IntakeConstants.KD, IntakeConstants.KFF);

        motorLeft.configure(mainConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        motorRight.configure(followerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        velocity = IntakeConstants.REST_VELOCITY;

        intakeController = motorLeft.getClosedLoopController();
        intakeController.setReference(velocity, SparkBase.ControlType.kVelocity);
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
        intakeController.setReference(velocity, SparkBase.ControlType.kVelocity);
    }

}
