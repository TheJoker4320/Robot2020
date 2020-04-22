package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeLifter extends SubsystemBase implements JokerSubsystem {
  private static IntakeLifter instance;
  private DoubleSolenoid solenoid;
  public boolean active=false;

  public static IntakeLifter getInstance() {
    if (instance == null)
      instance = new IntakeLifter();
    return instance;
  }

  private IntakeLifter() {
    //canLift = true;
    solenoid = new DoubleSolenoid(IntakeConstants.LIFTER_SELENOID_PORT[0], IntakeConstants.LIFTER_SELENOID_PORT[1]);
  }

  public void setSolenoidMode() {
    if(active){
      solenoid.set(Value.kReverse);
    }
    else{
      solenoid.set(Value.kForward);
    }
  }

  public void setState(boolean state){
    active = state;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public boolean isActive() {
    return active;
  }

  @Override
  public void disable() {
    active = false;
    solenoid.set(Value.kForward);
  }
}
