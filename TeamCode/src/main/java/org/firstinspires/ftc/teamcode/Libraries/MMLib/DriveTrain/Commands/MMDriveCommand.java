package org.firstinspires.ftc.teamcode.Libraries.MMLib.DriveTrain.Commands;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;

/**
 * this class runs the
 * {@link org.firstinspires.ftc.teamcode.Libraries.MMLib.DriveTrain.Subsystem.MMDriveTrain#fieldOrientedDrive(double, double, double) fieldOrientedDrive()}
 * with {@link GamepadEx gamepadEx1}.
 */
public class MMDriveCommand extends RunCommand {

    public MMDriveCommand() {
        super(
                () -> MMRobot.getInstance().mmSystems.mmDriveTrain.fieldOrientedDrive(
                        -MMRobot.getInstance().mmSystems.gamepadEx1.getLeftX(),
                        -MMRobot.getInstance().mmSystems.gamepadEx1.getLeftY(),
                        -MMRobot.getInstance().mmSystems.gamepadEx1.getRightX()
                ),
                MMRobot.getInstance().mmSystems.mmDriveTrain
        );
    }
}