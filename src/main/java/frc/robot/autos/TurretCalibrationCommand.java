package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.vision.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretCalibrationCommand extends Command {
  private final TurretSubsystem Turret;
  private final VisionSubsystem Pose;
  public double hoodTunerNumber;
  public double flywheelTunerNumber;
  public long waitTimeNumber;
  private Translation2d hubPosition = new Translation2d(4.625, 4.035);

  private static final String keyHood = "Tuning/Hood Angle (Deg)"; //the angle of the hood
  private static final String keyFlywheel = "Tuning/Flywheel RPS"; //current flywheel spd
  private static final String keyWaitTime = "Tuning/Wait Time (killiam william)";

  public TurretCalibrationCommand(TurretSubsystem turret, VisionSubsystem pose) {
    Turret = turret;
    Pose = pose;
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber(keyHood, 0);
    SmartDashboard.putNumber(keyFlywheel, 8.5);
    SmartDashboard.putNumber(keyWaitTime, 20);
  }

  @Override
  public void execute() {
    hoodTunerNumber = SmartDashboard.getNumber(keyHood, 0);
    flywheelTunerNumber = SmartDashboard.getNumber(keyFlywheel, 8.5);
    waitTimeNumber = (long) SmartDashboard.getNumber(keyWaitTime, 20);

    SmartDashboard.putNumber("Flywheel Speed", Turret.getFlywheelSpeed());
    SmartDashboard.putNumber("Feeder Speed: ", Turret.getFeederSpeed());
    SmartDashboard.putNumber("Distance from Hub: ", Pose.getDistToTarget(hubPosition));
    SmartDashboard.putNumber("Turret Angle", Turret.getTurretAngle());
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Turret.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
