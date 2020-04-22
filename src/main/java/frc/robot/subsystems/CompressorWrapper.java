package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CompressorWrapper extends SubsystemBase implements JokerSubsystem{
  private static CompressorWrapper instance;
  private Compressor madhes;
  private boolean active = false;

  public static CompressorWrapper getInstance() {
    if (instance == null)
      instance = new CompressorWrapper();
    return instance;
  }

  private CompressorWrapper() {
    madhes = new Compressor();
  }

  public boolean isActive() {
    return this.active;
  }

  public void setState(boolean state) {
    this.active = state;
  }

  public void activate(boolean state) {
    madhes.setClosedLoopControl(state);
  }

  public void disable() {
    madhes.setClosedLoopControl(false);
  }

  @Override
  public void periodic() {
  }
}
