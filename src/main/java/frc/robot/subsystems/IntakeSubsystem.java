package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {

    public boolean isAgitating = false;
    public boolean isIntaking = false;
    
    // Safety constants
    private final double MAX_ALLOWED_RACK_DELTA = 2.0; // Rotations
    private final double SAFE_RETRACT_POSITION = -0.05;
    private boolean m_safetyTripped = false;

    private final TalonFX m_intakeL = new TalonFX(29, "Aux");
    private final TalonFX m_intakeR = new TalonFX(28, "Aux");

    private final TalonFX m_intakeRL = new TalonFX(6, "Aux"); // rack left
    private final TalonFX m_intakeRR = new TalonFX(7, "Aux"); // rack right

    private final Follower m_intakeFollowRequest = new Follower(29, MotorAlignmentValue.Opposed);

    private final VelocityVoltage m_velocity = new VelocityVoltage(0);
    private final MotionMagicVoltage intakeRackMotionMagic = new MotionMagicVoltage(0).withSlot(0);
    
    public IntakeSubsystem() {
        final VoltageConfigs voltageConfigs = new VoltageConfigs();
        voltageConfigs.PeakForwardVoltage = 12.0; 
        voltageConfigs.PeakReverseVoltage = -12.0; 
        
        var talonFXconfigs = new TalonFXConfiguration();
        talonFXconfigs.Slot0.kP = 0.12;
        talonFXconfigs.Slot0.kV = 0.14;
        talonFXconfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        talonFXconfigs.CurrentLimits.StatorCurrentLimit = 60;
        talonFXconfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXconfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talonFXconfigs.Feedback.SensorToMechanismRatio = 1.0;
        m_intakeL.getConfigurator().apply(talonFXconfigs);

        final TalonFXConfiguration intakeRConfig = new TalonFXConfiguration();
        intakeRConfig.Feedback.SensorToMechanismRatio = 1.0;
        intakeRConfig.MotionMagic.MotionMagicCruiseVelocity = 20.0;
        intakeRConfig.MotionMagic.MotionMagicAcceleration = 36.0;  
        intakeRConfig.Slot0.kP = 0.34; 
        intakeRConfig.Slot0.kD = 0.0;
        intakeRConfig.Slot0.kV = 0.5;
        
        intakeRConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        intakeRConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 33.75; 
        intakeRConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        intakeRConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.05;
        intakeRConfig.CurrentLimits.StatorCurrentLimit = 35;
        intakeRConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeRConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        intakeRConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_intakeRL.getConfigurator().apply(intakeRConfig);

        intakeRConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_intakeRR.getConfigurator().apply(intakeRConfig);

        m_intakeL.getConfigurator().apply(voltageConfigs);
        m_intakeR.getConfigurator().apply(voltageConfigs);

        m_intakeRL.getConfigurator().apply(voltageConfigs); 
        m_intakeRR.getConfigurator().apply(voltageConfigs);
        
        m_intakeRL.setPosition(0);
        m_intakeRR.setPosition(0);
    }

    public void setIntakeVelocity(double rps) {
        m_intakeL.setControl(m_velocity.withVelocity(rps));
        m_intakeR.setControl(m_intakeFollowRequest);
    }

    public void applyIntakeRackVelocity(double rps) {
        m_intakeRL.setControl(m_velocity.withVelocity(rps));
        m_intakeRR.setControl(m_velocity.withVelocity(rps));
    }

    public double calculatedIntakeSpeeds(ChassisSpeeds fieldRelativeSpeeds) {
        double robotVelocity = Math.sqrt((fieldRelativeSpeeds.vxMetersPerSecond * fieldRelativeSpeeds.vxMetersPerSecond) + (fieldRelativeSpeeds.vyMetersPerSecond * fieldRelativeSpeeds.vyMetersPerSecond));
        double robotVelocityInFeet = robotVelocity * 3.281;
        double targetIntakeVelocity = robotVelocityInFeet * 2;
        return Math.max(70, targetIntakeVelocity * 2 * Math.PI);
    }

    private void applyIntakeVelocity(double rps) {
        m_intakeL.setControl(m_velocity.withVelocity(rps));
        m_intakeR.setControl(m_intakeFollowRequest);
    }
    
    public void zeroIntakeRollerVoltage() {
        this.m_intakeL.setVoltage(0);
        this.m_intakeR.setControl(m_intakeFollowRequest);
    }

    public void zeroIntakeVerticalVoltage() {
        this.m_intakeRL.setVoltage(0);
        this.m_intakeRR.setVoltage(0);
    }
    
    private void applyIntakePosition(double position) {
        // If safety is tripped, ignore requested position and go to safe retraction
        double target = m_safetyTripped ? SAFE_RETRACT_POSITION : position;
        m_intakeRL.setControl(intakeRackMotionMagic.withPosition(target));
        m_intakeRR.setControl(intakeRackMotionMagic.withPosition(target));
    }

    public Command setIntakeRackPosition(double position) {
        return this.runEnd(
            () -> this.applyIntakePosition(position),
            () -> zeroIntakeVerticalVoltage()
        );
    }
    
    public Command intakeOpen() {
        return this.runOnce(() -> this.applyIntakePosition(29.5));
    }
    
    public Command intakeClosed() {
        return this.run(() -> {
            this.applyIntakePosition(SAFE_RETRACT_POSITION);
            this.setIntakeVelocity(30);
        })
        .until(() -> Math.abs(m_intakeRL.getPosition().getValueAsDouble() - SAFE_RETRACT_POSITION) < 0.1)
        .finallyDo((interrupted) -> {
            this.setIntakeVelocity(0);
        });
    }

    public Command intakePositionZeroVolts() {
        return this.runOnce(() -> zeroIntakeVerticalVoltage());
    }

    private Command agitateStage(double basePos, double altPos, double duration) {
        Timer timer = new Timer();
        return this.run(() -> {
            double currentPos = m_intakeRL.getPosition().getValueAsDouble();
            
            if (Math.abs(currentPos - basePos) < 1.0) {
                if (!timer.isRunning()) {
                    timer.restart(); 
                }
                double setpoint = (timer.get() % 0.6 < 0.3) ? basePos : altPos;
                this.applyIntakePosition(setpoint);
            } else {
                this.applyIntakePosition(basePos);
            }
            this.setIntakeVelocity(20);
        })
        .beforeStarting(timer::stop)
        .until(() -> timer.isRunning() && timer.hasElapsed(duration));
    }

    public Command intakeAgitate() {
        return agitateStage(30.0, 24.0, 4.0)
            .andThen(agitateStage(24.0, 16.0, 4.0))
            .andThen(agitateStage(-0.05, 3.75, 20.0))
            .finallyDo((interrupted) -> {
                this.setIntakeVelocity(0);
                this.applyIntakePosition(-0.05);
            });
    }

    public Command intakeOutAndIntakeCommand(Supplier<ChassisSpeeds> chassisSpeeds) {
        return this.runEnd(() -> {
            double targetSpeed = calculatedIntakeSpeeds(chassisSpeeds.get());
            applyIntakeVelocity(targetSpeed);  
            isIntaking = true;
        }, 
        () -> {
            setIntakeVelocity(0);
            zeroIntakeVerticalVoltage();
            isIntaking = false;
        });
    }
    
    public Command slowlyAgitateAndSpinCommand() {  
        Timer unjamTimer = new Timer();
        
        // We use an array to store mutable state inside the lambda. 
        // state[0] = Current Target Position
        // state[1] = Is currently unjamming? (0.0 for false, 1.0 for true)
        double[] state = new double[2]; 

        // --- TUNE THESE VALUES ---
        // The current spike (in Amps) that counts as a "jam". 
        // Check your telemetry to see what the rack normally pulls while moving empty, and set this higher.
        final double JAM_CURRENT_THRESHOLD = 30.0; 
        
        // How many rotations to back up when jammed
        final double BACKUP_AMOUNT = 4.5; 
        
        // How long to pause and let the rack back up before trying to pull in again (seconds)
        final double UNJAM_TIME = 0.3; 

        return this.runEnd(() -> {
            boolean isUnjamming = state[1] > 0.5;

            if (!isUnjamming) {
                state[0] -= 0.1; 
                
                // Detect jam
                if (m_intakeRL.getStatorCurrent().getValueAsDouble() > JAM_CURRENT_THRESHOLD) {
                    state[1] = 1.0; // Trigger unjam state
                    state[0] += BACKUP_AMOUNT; // Move target backwards
                    unjamTimer.restart();
                }
            } else {
                if (unjamTimer.hasElapsed(UNJAM_TIME)) {
                    state[1] = 0.0; // Resume normal operation
                    unjamTimer.stop();
                }
            }

            state[0] = Math.max(state[0], SAFE_RETRACT_POSITION);
            
            applyIntakePosition(state[0]);
            applyIntakeVelocity(15.0);
        },
        () -> setIntakeVelocity(0))
        .beforeStarting(() -> {
            state[0] = m_intakeRL.getPosition().getValueAsDouble();
            state[1] = 0.0;
            unjamTimer.stop();
            unjamTimer.reset();
        });
    }

    public Command intakeDownAndOuttakeCommand() {
        return this.runEnd(
            () -> {
                applyIntakeVelocity(-50);
                zeroIntakeVerticalVoltage();
            },
            () -> {
                applyIntakeVelocity(0);
                zeroIntakeVerticalVoltage();
            });
    }
    
    public Command beginIntakeCommand(Supplier<ChassisSpeeds> chassisSpeed) {
        return this.run(() -> {
            this.setIntakeVelocity(calculatedIntakeSpeeds(chassisSpeed.get()));
            this.applyIntakePosition(29.5);
        });
    }

    public Command endIntakeCommand() {
        return this.runOnce(() -> {
            this.m_intakeL.stopMotor();
            this.m_intakeR.setControl(m_intakeFollowRequest);
            zeroIntakeVerticalVoltage();
        });
    }

    public Command beginOutakeCommand() {
        return this.run(() -> {
            this.setIntakeVelocity(-40);
        });
    }

    public Command endOutakeCommand() {
        return this.runOnce(() -> {
            this.m_intakeL.stopMotor();
            this.m_intakeR.setControl(m_intakeFollowRequest);
            zeroIntakeVerticalVoltage();
        });
    }

    public double getIntakeVelocity() {
        return m_intakeL.getVelocity().getValueAsDouble();
    } 

    @Override
    public void periodic() {
        double leftPos = m_intakeRL.getPosition().getValueAsDouble();
        double rightPos = m_intakeRR.getPosition().getValueAsDouble();
        double delta = Math.abs(leftPos - rightPos);

        // Sync Safety Logic
        if (delta > MAX_ALLOWED_RACK_DELTA) {
            if (!m_safetyTripped) {
                System.err.println("CRITICAL: Intake Rack Desync! Delta: " + delta);
                m_safetyTripped = true;
            }
        } else if (m_safetyTripped && delta < 0.1) {
            // Auto-reset safety if they get back in sync
            m_safetyTripped = false;
        }

        SmartDashboard.putNumber("Rack/Left Pos", leftPos);
        SmartDashboard.putNumber("Rack/Right Pos", rightPos);
        SmartDashboard.putNumber("Rack/Delta", delta);
        SmartDashboard.putBoolean("Rack/Safety Tripped", m_safetyTripped);
    }
}