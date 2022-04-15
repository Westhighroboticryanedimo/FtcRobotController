package org.firstinspires.ftc.teamcode.reee;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Controller;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.teamcode.utils.localization.Point;

import org.firstinspires.ftc.teamcode.reee.LocalizerReee;

@TeleOp(name="dash reee", group="reee")

public class dash extends OpMode {
    LocalizerReee localizer;
    Controller controller;
    FtcDashboard dash = FtcDashboard.getInstance();

    @Override
    public void init() {
        localizer = new LocalizerReee(this, hardwareMap);
    }

    @Override
    public void loop() {
        if (controller.AOnce()) {
            localizer.scan();
            localizer.correctToField();
        } else if (controller.BOnce()) {
            localizer.scan();
            localizer.correctDists();
            localizer.correctToField();
        }
        TelemetryPacket packet = new TelemetryPacket();
        for (int i = 0; i < localizer.getMap().size(); ++i) {
            packet.fieldOverlay()
                    .setFill("blue")
                    .fillRect(localizer.getMap().get(i).x, localizer.getMap().get(i).y, 2, 2);
            dash.sendTelemetryPacket(packet);
        }
    }

}
