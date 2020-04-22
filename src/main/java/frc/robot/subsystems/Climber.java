package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private static Climber instance;
  private WPI_VictorSPX climberMasterMotor;
  public boolean active = false;

  public static Climber getInstance() {
    if (instance == null)
      instance = new Climber();
    return instance;
  }

  private Climber() {
    climberMasterMotor = new WPI_VictorSPX(ClimberConstants.CLIMBER_MOTOR_PORTS);
  }

  public void setOutput(double percentage) {
    climberMasterMotor.set(percentage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
