package frc.robot.subsystems;

import frc.robot.utils.*;
import frc.robot.vision.VisionSubsystem;
import frc.robot.subsystems.*;
import frc.robot.autos.*;
import frc.robot.constants.TurretConstants;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.filter.LinearFilter; 
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.*;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TurretSubsystem extends SubsystemBase {

    public boolean aimingToggle = false;
    public boolean isShooting = false;

    public VisionSubsystem Pose;
    public CommandSwerveDrivetrain Drivetrain;
    public Translation2d targetHub = new Translation2d(4.625, 4.035);

    private final TalonFX turretTurner = TurretConstants.turretTurner;
    private final TalonFX turretShooter = TurretConstants.turretShooter;
    private final TalonFX turretFeeder = TurretConstants.turretFeeder;
    private final TalonFX turretHood = TurretConstants.turretHood;
    private final TalonFX turretSpindexer = TurretConstants.turretSpindexer;
    
    private final CANcoder enc10T = TurretConstants.encoder10T;
    private final CANcoder enc11T = TurretConstants.encoder11T;

    private final VelocityVoltage velocity = new VelocityVoltage(0);
    private final com.ctre.phoenix6.controls.VoltageOut zeroVolts = new com.ctre.phoenix6.controls.VoltageOut(0);
    private final com.ctre.phoenix6.controls.VoltageOut sysIdControl = new com.ctre.phoenix6.controls.VoltageOut(0);
    private final MotionMagicVoltage turnerMMRequest = new MotionMagicVoltage(0); 
    private final MotionMagicVoltage hoodMMRequest = new MotionMagicVoltage(0); 
    public InterpolatingDoubleTreeMap m_shooterSpeedMap = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap m_hoodAngleMap = new InterpolatingDoubleTreeMap();
    
    private final double shooterWheelRadius = Units.inchesToMeters(2.0); 
    
    private final double fuelEfficiency = 0.4; 
    private final double maxShift = 2;
    private final double rotationLookAhead = 0.1;
    private final double TURRET_X_OFFSET = 0.106;
    private final double TURRET_Y_OFFSET = 0.106;
    private final Translation2d robotToTurretTranslation = new Translation2d(TURRET_X_OFFSET, TURRET_Y_OFFSET);
    private double currentAimTargetRotations = 0;

    private final SimpleCRT crtSolver = new SimpleCRT(10, 11, 100, -10.0, 360.0);
    
    public TurretSubsystem() {
        seedDistMaps();
        configureMotors();

        try { Thread.sleep(250); } catch (InterruptedException e) {}

        seedTurretPosition();

        turretHood.setPosition(0.0);
    }

    private final SysIdRoutine shooterSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, 
            Volts.of(4),
            null,
            state -> SignalLogger.writeString("ShooterTestState", state.toString())
        ), 
        new SysIdRoutine.Mechanism(
            (voltage) -> turretShooter.setControl(sysIdControl.withOutput(voltage.in(Volts))),
            (log) -> {
                log.motor("ShooterFlywheel")
                   .voltage(Volts.of(turretShooter.getMotorVoltage().getValueAsDouble()))
                   .angularPosition(Rotations.of(turretShooter.getPosition().getValueAsDouble()))
                   .angularVelocity(RotationsPerSecond.of(turretShooter.getVelocity().getValueAsDouble()));
            },
            this
        )
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return shooterSysIdRoutine.quasistatic(direction)
            .beforeStarting(SignalLogger::start)
            .finallyDo(SignalLogger::stop);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return shooterSysIdRoutine.dynamic(direction)
            .beforeStarting(SignalLogger::start)
            .finallyDo(SignalLogger::stop);
    }

    public void setDependencies(VisionSubsystem pose, CommandSwerveDrivetrain drivetrain) {
        this.Pose = pose;
        this.Drivetrain = drivetrain;
    }

    public boolean getAimingToggle() {
        return aimingToggle;
    }

    public void configureMotors() {
        final VoltageConfigs voltageConfigs = new VoltageConfigs();
        voltageConfigs.PeakForwardVoltage = 12.0; 
        voltageConfigs.PeakReverseVoltage = -12.0; 

        final TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.Feedback.SensorToMechanismRatio = 1.6;
        flywheelConfig.Slot0.kV = 0.2725;
        flywheelConfig.Slot0.kP = 0.6;
        flywheelConfig.Slot0.kD = 0.002;
        flywheelConfig.Slot0.kS = 0.06;
        flywheelConfig.Slot0.kA = 0.015;
        flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
        
        final TalonFXConfiguration feederConfig = new TalonFXConfiguration();
        feederConfig.Slot0.kP = 0.35;
        feederConfig.Slot0.kV = 0.098;
        feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        feederConfig.CurrentLimits.StatorCurrentLimit = 60.0;
        feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        final TalonFXConfiguration turnerConfig = new TalonFXConfiguration();
        turnerConfig.Feedback.SensorToMechanismRatio = 125.0/3.0; 
        turnerConfig.MotionMagic.MotionMagicCruiseVelocity = 6.0;
        turnerConfig.MotionMagic.MotionMagicAcceleration = 20.0;  
        turnerConfig.MotionMagic.MotionMagicJerk = 100.0;
        turnerConfig.Slot0.kP = 80; 
        turnerConfig.Slot0.kV = 0.35;
        turnerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        turnerConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0; 
        turnerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        turnerConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -1.0;
        turnerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        turnerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turnerConfig.CurrentLimits.StatorCurrentLimit = 60.0;
        
        final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 12.0; 
        hoodConfig.MotionMagic.MotionMagicAcceleration = 24.0;
        hoodConfig.Slot0.kV = 0.1175;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 13;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.5; 

        final TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
        spindexerConfig.Feedback.SensorToMechanismRatio = 27.0;
        spindexerConfig.Slot0.kP = 0;
        spindexerConfig.Slot0.kV = 0.1;
        spindexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        spindexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        spindexerConfig.CurrentLimits.StatorCurrentLimit = 60.0;
        spindexerConfig.CurrentLimits.SupplyCurrentLimit = 60.0;

        final CANcoderConfiguration enc10TConfiguration = new CANcoderConfiguration();
        enc10TConfiguration.MagnetSensor.MagnetOffset = 0.56;
        enc10TConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        enc10TConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        final CANcoderConfiguration enc11TConfiguration = new CANcoderConfiguration();
        enc11TConfiguration.MagnetSensor.MagnetOffset = 0.85;
        enc11TConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        enc11TConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;


        turretTurner.getConfigurator().apply(turnerConfig);
        turretTurner.getConfigurator().apply(voltageConfigs);

        turretShooter.getConfigurator().apply(flywheelConfig);
        turretShooter.getConfigurator().apply(voltageConfigs);

        turretFeeder.getConfigurator().apply(feederConfig);
        turretFeeder.getConfigurator().apply(voltageConfigs);

        turretHood.getConfigurator().apply(hoodConfig);
        turretHood.getConfigurator().apply(voltageConfigs);

        turretSpindexer.getConfigurator().apply(spindexerConfig);
        turretSpindexer.getConfigurator().apply(voltageConfigs);

        enc10T.getConfigurator().apply(enc10TConfiguration);
        enc11T.getConfigurator().apply(enc11TConfiguration);
    }

    private void seedTurretPosition() {
        double posA = enc10T.getAbsolutePosition().getValueAsDouble(); 
        double posB = enc11T.getAbsolutePosition().getValueAsDouble();

        System.out.println(String.format("RAW: %.4f | %.4f", posA, posB));

        double solvedAngle = crtSolver.getTrueAngle(posA, posB);

        if (Double.isNaN(solvedAngle)) {
            System.out.println("CRT FAILURE: Sensors disagree or are out of bounds!");
            turretTurner.setPosition(0.0); 
        } else {
            System.out.println("CRT SUCCESS -> Angle: " + String.format("%.2f", solvedAngle));
            
            turretTurner.setPosition(-solvedAngle / 360.0);
        }
    }

    public void seedDistMaps() {
        m_shooterSpeedMap.put(1.75, 12.45);
        m_hoodAngleMap.put(1.75, 0.0);
        m_shooterSpeedMap.put(2.0, 12.55);
        m_hoodAngleMap.put(2.0, 0.0);
        m_shooterSpeedMap.put(2.5, 12.75);
        m_hoodAngleMap.put(2.5, 3.0);
        m_shooterSpeedMap.put(3.0, 12.75);
        m_hoodAngleMap.put(3.0, 6.0);
        m_shooterSpeedMap.put(3.5, 12.95);
        m_hoodAngleMap.put(3.5, 9.0);
        m_shooterSpeedMap.put(4.0, 12.95);
        m_hoodAngleMap.put(4.0, 10.0);
        m_shooterSpeedMap.put(4.5, 12.95);
        m_hoodAngleMap.put(4.5, 11.0);
        m_shooterSpeedMap.put(5.0, 12.95);
        m_hoodAngleMap.put(5.0, 11.0);
        m_shooterSpeedMap.put(5.5, 13.1);
        m_hoodAngleMap.put(5.5, 11.0);
    }

    public double autoAim(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds, Supplier<Translation2d> targetPosition) {

        Translation2d turretOffsetField = robotToTurretTranslation.rotateBy(robotPose.getRotation());
        Translation2d turretLocationField = robotPose.getTranslation().plus(turretOffsetField);

        double omega = fieldRelativeSpeeds.omegaRadiansPerSecond;
        Translation2d tangentialVelocity = new Translation2d(-omega * turretOffsetField.getY(), omega * turretOffsetField.getX());

        double totalVx = fieldRelativeSpeeds.vxMetersPerSecond + tangentialVelocity.getX();
        double totalVy = fieldRelativeSpeeds.vyMetersPerSecond + tangentialVelocity.getY();

        double dragFactor = 0.7; 
        double effectiveVx = totalVx * dragFactor;
        double effectiveVy = totalVy * dragFactor;

        double targetX = targetPosition.get().getX();
        double targetY = targetPosition.get().getY();
        double virtX = targetX;
        double virtY = targetY;
    
        double distance = turretLocationField.getDistance(targetPosition.get());
        double t = calculateTimeOfFlight(distance);

        for (int i = 0; i < 3; i++) {
            double shiftX = -(effectiveVx * t);
            double shiftY = -(effectiveVy * t);
        
            shiftX = MathUtil.clamp(shiftX, -maxShift, maxShift);
            shiftY = MathUtil.clamp(shiftY, -maxShift, maxShift);

            virtX = targetX + shiftX;
            virtY = targetY + shiftY;
        
            double dx = virtX - turretLocationField.getX();
            double dy = virtY - turretLocationField.getY();
            distance = Math.sqrt((dx * dx) + (dy * dy));
        
            t = calculateTimeOfFlight(distance);
            if (t > 2.0) t = 2.0; 
        }

        double dx = virtX - turretLocationField.getX();
        double dy = virtY - turretLocationField.getY();
        double targetAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
    
        double relativeAngle = targetAngleDeg - robotPose.getRotation().getDegrees();
        double aimAngle = Math.IEEEremainder(relativeAngle + 125, 360.0);

        double robotSpinRPS = omega / (2 * Math.PI);
        double finalTarget = aimAngle + (-robotSpinRPS * rotationLookAhead * 360.0);

        finalTarget = MathUtil.inputModulus(finalTarget, -360, 0);

        moveTurretAngle(finalTarget / 360.0);
    
        return distance;
    }

    public double calculateTimeOfFlight(double distance) {
        double targetRPS = m_shooterSpeedMap.get(distance);
        double targetHoodDeg = m_hoodAngleMap.get(distance); 

        double flywheelSurfaceSpeed = targetRPS * (2 * Math.PI * shooterWheelRadius);
        double exitVelocity = flywheelSurfaceSpeed * fuelEfficiency;

        double horizontalVelocity = exitVelocity * Math.cos(Math.toRadians(targetHoodDeg));

        if (horizontalVelocity < 1.0) return 1.0; 
        return (distance / horizontalVelocity) + 0.15;
    }

    public boolean isShooterAtSpeed(double targetRPS) {
        return Math.abs(turretShooter.getVelocity().getValueAsDouble() - targetRPS) < 1.0;
    }

    public boolean isHoodAtAngle(double targetAngle) {
        return Math.abs(turretHood.getPosition().getValueAsDouble() - targetAngle) < 3;
    }

    public boolean isTurretAligned(double toleranceDegrees){
        double currentAimedPosition = turretTurner.getPosition().getValueAsDouble();
        double error = Math.abs(currentAimedPosition - currentAimTargetRotations) * 360;
        return error < toleranceDegrees;
    }

    public void toggleAiming(){
        aimingToggle = !aimingToggle;
    }

    public Command getAutoAimAndShootCommand(VisionSubsystem pose, CommandSwerveDrivetrain drivetrain, Supplier<Translation2d> targetHub, Supplier<Boolean> nearTrench) {
        return this.runEnd(
            () -> {
                isShooting = true;
                double virtualDist = autoAim(pose.getCurrentPose(), drivetrain.getFieldRelativeSpeed(), targetHub);
            
                double targetSpeed = m_shooterSpeedMap.get(virtualDist);
                double targetAngle = m_hoodAngleMap.get(virtualDist);
                setShooterVelocity(targetSpeed);

                if (!nearTrench.get()){
                    setHoodPosition(targetAngle);
                } else {
                    setHoodPosition(-0.2);
                }
                
                if (isShooterAtSpeed(targetSpeed) && isHoodAtAngle(targetAngle)) {
                    setFeederVelocity(100); 
                    setSpindexerVelocity(50); //80
                } else {
                   slowFeeding();
                }
            },
            () -> {
                isShooting = false;
                stopMotors();
                //setShooterVelocity(5.0); //5.0
                stopFeeding();
                setHoodPosition(-0.3);
            }
        );
    }

    public Command getAutoAimAndShootCommandCalibration(VisionSubsystem pose, CommandSwerveDrivetrain drivetrain, Supplier<Translation2d> targetHub, boolean nearTrench, DoubleSupplier hoodPosition, DoubleSupplier flywheelPower) {
        return this.runEnd(
            () -> {
                isShooting = true;
                double virtualDist = autoAim(pose.getCurrentPose(), drivetrain.getFieldRelativeSpeed(), targetHub);
            
                double targetSpeed = m_shooterSpeedMap.get(virtualDist);

                setShooterVelocity(flywheelPower.getAsDouble());
                
                if (!nearTrench){
                    setHoodPosition(hoodPosition.getAsDouble());
                } else {
                    setHoodPosition(0);
                }

                if (isShooterAtSpeed(flywheelPower.getAsDouble()) && isTurretAligned(4)) {
                    setFeederVelocity(100);
                    setSpindexerVelocity(50);
                } else {
                    slowFeeding();
                }
            },
            () -> {
                isShooting = false;
                stopMotors();
                //setShooterVelocity(5.0);
                stopFeeding();
                setHoodPosition(-0.3);
            }
        );
    }

    public Command getAutoAimAndShoot(VisionSubsystem pose, CommandSwerveDrivetrain drivetrain, Supplier<Translation2d> targetHub, Supplier<Boolean> nearTrench) {
        return this.runEnd(
            () -> {
                isShooting = true;
                double virtualDist = autoAim(pose.getCurrentPose(), drivetrain.getFieldRelativeSpeed(), targetHub);
            
                double targetSpeed = m_shooterSpeedMap.get(virtualDist);
                double targetAngle = m_hoodAngleMap.get(virtualDist); 
                setShooterVelocity(targetSpeed); //target speed

                if (!nearTrench.get()){
                    setHoodPosition(targetAngle); //target angle
                } else {
                    setHoodPosition(-0.2);
                }
                
                if (isShooterAtSpeed(targetSpeed) && isHoodAtAngle(targetAngle)) {
                    setFeederVelocity(110); 
                    setSpindexerVelocity(70); //70
                } else {
                    slowFeeding();    
                }
            },
            () -> {
                isShooting = false;
                stopMotors();
                turretShooter.stopMotor();
                //setShooterVelocity(0.0);
                stopFeeding();
                setHoodPosition(-0.3);
            }
        );
    }

    public void moveTurretAngle(double turretRotations) {
        this.currentAimTargetRotations = turretRotations;
        turretTurner.setControl(turnerMMRequest.withPosition(turretRotations));
    }
    
    public void stopMotors() {
        turretTurner.setControl(zeroVolts);
        turretFeeder.setControl(zeroVolts);
        turretSpindexer.setControl(zeroVolts);
    }

    public void stopFeeding() {
        turretFeeder.setControl(zeroVolts);
        turretSpindexer.setControl(zeroVolts);
    }

    public void slowFeeding() {
        setFeederVelocity(20);
        setSpindexerVelocity(30);
    }

    public void setShooterVelocity(double targetRPS) {
        turretShooter.setControl(velocity.withVelocity(targetRPS));
    }

    public void setFeederVelocity(double rps) {
        turretFeeder.setControl(velocity.withVelocity(rps));
    }

    public void setHoodPosition(double position) {
        turretHood.setControl(hoodMMRequest.withPosition(position));
    }

    public void setSpindexerVelocity(double rps) {
        turretSpindexer.setControl(velocity.withVelocity(rps));
    }

    public double getFeederSpeed(){
        return turretFeeder.getVelocity().getValueAsDouble();
    }

    public double getFlywheelSpeed(){
        return turretShooter.getVelocity().getValueAsDouble();
    }

    public double getTurretAngle() {
        return turretTurner.getPosition().getValueAsDouble() * 360;
    }

    public double getSpindexerSpeed()
    {
        return turretSpindexer.getVelocity().getValueAsDouble();
    }

    public double getFlywheelStatorCurrent() {
        return turretShooter.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret/Heartbeat", Timer.getFPGATimestamp());
        
        if (Pose == null) return;

        double distance = Pose.getCurrentPose().getTranslation().getDistance(new Translation2d(11.925, 4.04)); //red hub
        double currentTargetRPS = m_shooterSpeedMap.get(distance);
    
        boolean atSpeed = isShooterAtSpeed(currentTargetRPS);
        
        SmartDashboard.putBoolean("Turret/Is Shooter At Speed?", atSpeed);
    }
}