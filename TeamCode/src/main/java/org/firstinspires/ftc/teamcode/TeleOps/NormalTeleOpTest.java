package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMPIDCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMTuningFFCommand;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;

@TeleOp
public class NormalTeleOpTest extends MMTeleOp {
    MMRobot robot = MMRobot.getInstance();

    public NormalTeleOpTest(){
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    @Override
    public void onInit() {

        MMRobot.getInstance().mmSystems.initElevator();

        //creating custom command-bindings for tuning: (using dashboard)

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new MMPIDCommand( //up to 15cm
                        MMRobot.getInstance().mmSystems.elevator,
                        15
                )
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new MMPIDCommand( //back to 0
                        MMRobot.getInstance().mmSystems.elevator,
                        0
                )
        );

        //creating custom FF tuning command: (using telemetry)

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new MMTuningFFCommand( //minimal power to reach 1cm
                        MMRobot.getInstance().mmSystems.elevator,
                        1
                )
        );
    }

    @Override
    public void run() {
        super.run();
    }
}