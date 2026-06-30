package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

@Logged
public class HopperSubsystem extends SubsystemBase {
  private final VictorSPX hopperMotor = new VictorSPX(Constants.HopperConstants.kSpinMotorCanbusID);

  private GenericEntry nt_spinPercentage;

  private boolean isControllingForRPM = false;
  private boolean isSpinning = false;

  public HopperSubsystem() {
    this.initialize(); 
  }

  public void initialize() {
    setupShuffleboard();
  }

  public void startSpinning() {
    isControllingForRPM = true;
    isSpinning = true;
  }

  public void stopSpinning() {
    isControllingForRPM = false;
    isSpinning = false;
    hopperMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }

  public Command toggleSpin() {
    return new ConditionalCommand(
      new InstantCommand(() -> startSpinning()),
      new InstantCommand(() -> stopSpinning()),
      () -> !isSpinning
    );
  }

  @Override
  public void periodic() {
    if(isControllingForRPM){
      hopperMotor.set(VictorSPXControlMode.PercentOutput, getTargetPercentage());
    }
  }

  public double getTargetPercentage() {
    return nt_spinPercentage.getDouble(Constants.HopperConstants.kSpinPercentage);
  }

  public void setupShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Hopper");

    nt_spinPercentage = tab.addPersistent("Hopper Percentage", Constants.HopperConstants.kSpinPercentage)
      .getEntry();
  }
}