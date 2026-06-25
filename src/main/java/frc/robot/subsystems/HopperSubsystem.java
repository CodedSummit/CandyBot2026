package frc.robot.subsystems;

import java.util.Map;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

@Logged
public class HopperSubsystem extends SubsystemBase {
  private final VictorSPX hopperMotor = new VictorSPX(Constants.HopperConstants.kSpinMotorCanbusID);

  private GenericEntry nt_hopperSpeed;
  private GenericEntry nt_hopperGainRPM;
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
    hopperMotor.set(VictorSPXControlMode.PercentOutput, 0.3);
  }

  public void stopSpinning() {
    isControllingForRPM = false;
    isSpinning = false;
    hopperMotor.set(VictorSPXControlMode.Disabled, 0);
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

  public double getGain() {
    return nt_hopperGainRPM.getDouble(Constants.HopperConstants.kSpinGain);
  }

  public double getShooterSpeed() {
    return nt_hopperSpeed.getDouble(Constants.HopperConstants.kSpinSpeed);
  }

  public double getTargetPercentage() {
    return nt_spinPercentage.getDouble(Constants.HopperConstants.kSpinPercentage);
  }

  public void setupShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Hopper");

    nt_hopperSpeed = tab.addPersistent("Hopper Speed", Constants.HopperConstants.kSpinSpeed)
      .withSize(3, 1)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 1))
      .getEntry();

    nt_hopperGainRPM = tab.addPersistent("Feed Forward Gain", Constants.HopperConstants.kSpinGain)
      .getEntry();

    nt_spinPercentage = tab.addPersistent("Hopper Percentage", Constants.HopperConstants.kSpinPercentage)
      .getEntry();
  }
}