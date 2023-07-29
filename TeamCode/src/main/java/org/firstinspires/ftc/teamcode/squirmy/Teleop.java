package org.firstinspires.ftc.teamcode.squirmy;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.node.*;
import org.firstinspires.ftc.teamcode.squirmy.BulkReadNode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Squirmy TeleOp")
public class Teleop extends OpMode {

    NodeScheduler nodeScheduler = new NodeScheduler(new BulkReadNode(hardwareMap.getAll(LynxModule.class),
                                                            hardwareMap.get(DcMotorEx.class, "m1"),
                                                            hardwareMap.get(DcMotorEx.class, "m2"),
                                                            hardwareMap.get(DcMotorEx.class, "m3"),
                                                            hardwareMap.get(DcMotorEx.class, "m4")));
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        nodeScheduler.init();
    }

    @Override
    public void loop() {
        timer.reset();
        nodeScheduler.update();
        telemetry.update();
    }
}