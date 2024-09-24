package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;

public class LinearIntakeCommand extends CommandBase{

    public LinearIntakeCommand(){
        addRequirements(MMRobot.getInstance().mmSystems.linearIntake);
    }


    @Override
    public void execute() {
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(MMRobot.getInstance().mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
   //TODO: tuning
    }

    @Override
    public void end(boolean interrupted) {
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);
    }
}
