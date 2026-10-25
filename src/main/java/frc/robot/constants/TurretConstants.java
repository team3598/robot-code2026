// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class TurretConstants {
    public static final TalonFX turretTurner = new TalonFX(50, "Aux");
    public static final TalonFX turretShooter = new TalonFX(51, "Aux");
    public static final TalonFX turretFeeder = new TalonFX(3, "Aux");
    public static final TalonFX turretHood = new TalonFX(44, "Aux");

    public static final TalonFX turretSpindexer = new TalonFX(30, "Aux");

    public static final CANcoder encoder11T = new CANcoder(1, "Aux"); 
    public static final CANcoder encoder10T = new CANcoder(2, "Aux"); 
}