// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import java.util.function.Supplier;

/** Add your docs here. */
public class MotorSimConfiguration {

    public Supplier<DCMotor> simMotorModelSupplier = () -> DCMotor.getKrakenX60Foc(1);
    public double simReduction = 1;
    public double simMOI = 0.001;
}
