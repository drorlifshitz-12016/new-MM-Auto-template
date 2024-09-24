package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Commands.ClawSetState;
import org.firstinspires.ftc.teamcode.Commands.IntakeByToggle;
import org.firstinspires.ftc.teamcode.Commands.LinearIntakeCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.DriveTrain.Commands.ResetFieldOrientedCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMPIDCommand;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;

@TeleOp
public class TeleopDrive extends MMTeleOp {

    MMRobot robot = MMRobot.getInstance();
    public TeleopDrive() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        MMRobot.getInstance().mmSystems.initDriveTrain();
        MMRobot.getInstance().mmSystems.initIntake();
        MMRobot.getInstance().mmSystems.initLinearIntake();
        MMRobot.getInstance().mmSystems.initElevator();


        Trigger leftTriggerCondition = new Trigger(()-> gamepad1.left_trigger > 0.1);
        leftTriggerCondition.whenActive(
                new LinearIntakeCommand()
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new IntakeByToggle()
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new ResetFieldOrientedCommand()
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new ClawSetState(robot.mmSystems.claw,Claw.State.OPEN)
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new IntakeArmCommand(robot.mmSystems.armAngle, IntakeArm.Position.IN)
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new IntakeArmCommand(robot.mmSystems.armAngle, IntakeArm.Position.OUT)
        );


        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new MMPIDCommand( //up to 15cm
                        MMRobot.getInstance().mmSystems.elevator,
                        15
                )
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new MMPIDCommand(
                        MMRobot.getInstance().mmSystems.elevator,
                        0
                )
        );




    }

    @Override
    public void run() {
        super.run();
        telemetry.addData(
                "yaw",
                MMRobot.getInstance().mmSystems.imu.getYawInDegrees()
        );
        telemetry.addData("meow",MMRobot.getInstance().mmSystems.elevator.getHeight());
        telemetry.update();
        telemetry.addData("linerServo",MMRobot.getInstance().mmSystems.linearIntake.getPosition());
        telemetry.update();
    }

}