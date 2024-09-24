package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.RollerIntake;

public class IntakeByPower extends CommandBase {
    RollerIntake intake;
    double power;

    public IntakeByPower(double power){
        this.power = power;
        this.addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);
    }

}
