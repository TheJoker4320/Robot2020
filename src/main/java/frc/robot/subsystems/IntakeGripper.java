package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeGripper extends SubsystemBase implements JokerSubsystem {
  private static IntakeGripper instance;

  private final WPI_TalonSRX intakeMotor;
  public static enum States{
    BACKWARD,
    IDLE,
    FORWARD
  };
  private States active = States.IDLE;
  public boolean blocked = true;


  public static IntakeGripper getInstance() {
    if (instance == null)
      instance = new IntakeGripper();
    return instance;
  }

  private IntakeGripper() {
    intakeMotor = new WPI_TalonSRX(IntakeConstants.GRIPPER_MOTOR_PORT);
    this.intakeMotor.configFactoryDefault();
  }

  public States getState(){
    return this.active;
  }

  public boolean isActive(){
    return this.active.ordinal() != 0;
  }

  public void setState(States state){
    this.active = state;
  }

  public void setOutput(double percentage) {
    if(active == States.IDLE)
      intakeMotor.set(0);
    if(active == States.FORWARD)
      intakeMotor.set(percentage);
    if(active == States.BACKWARD)
      intakeMotor.set(-percentage);
  }

  @Override
  public void disable() {
    this.active = States.IDLE;
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
  }
}
