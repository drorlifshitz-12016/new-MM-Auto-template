package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;

public class IntakeArmCommand extends InstantCommand {


    public IntakeArmCommand(IntakeArm armAngle, IntakeArm.Position position){
        super(()-> armAngle.setPosition(position.intakeArmPosition));
    }
}
