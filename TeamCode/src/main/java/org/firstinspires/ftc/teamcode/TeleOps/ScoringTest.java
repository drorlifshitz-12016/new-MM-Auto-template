package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.Configuration;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;

@TeleOp
public class ScoringTest extends MMTeleOp {
    MMRobot robot = MMRobot.getInstance();

    CuttleServo servo;

    public ScoringTest() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    @Override
    public void onInit() {
        MMRobot.getInstance().mmSystems.initClaw();
        servo = new CuttleServo(robot.mmSystems.controlHub, Configuration.CLAW_SCORING);
    }



    @Override
    public void run() {
        super.run();
        servo.setPosition(gamepad1.left_trigger);
        telemetry.addData("pos",servo.getPosition());
        telemetry.addData("trig",robot.mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        telemetry.update();
    }
}
