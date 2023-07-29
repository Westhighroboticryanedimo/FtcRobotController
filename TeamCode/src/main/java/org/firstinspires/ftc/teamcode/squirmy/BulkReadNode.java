package org.firstinspires.ftc.teamcode.squirmy;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.node.Node;

import java.util.HashMap;
import java.util.List;

public class BulkReadNode extends Node {
    private DcMotorEx m1, m2, m3, m4;
    private int e1, e2, e3, e4;
    private double c1, c2, c3, c4;
    private List<LynxModule> allHubs;

    public BulkReadNode(List<LynxModule> hubs, DcMotorEx motor1, DcMotorEx motor2, DcMotorEx motor3, DcMotorEx motor4) {
        allHubs = hubs;
        m1 = motor1;
        m2 = motor2;
        m3 = motor3;
        m4 = motor4;

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void init() {
        loop();
    }

    @Override
    public void loop() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
        e1 = m1.getCurrentPosition();
        e2 = m2.getCurrentPosition();
        e3 = m3.getCurrentPosition();
        e4 = m4.getCurrentPosition();
        c1 = m1.getCurrent(CurrentUnit.AMPS);
        c2 = m2.getCurrent(CurrentUnit.AMPS);
        c3 = m3.getCurrent(CurrentUnit.AMPS);
        c4 = m4.getCurrent(CurrentUnit.AMPS);
    }

    @Override
    public HashMap<String, Object> publish() {
        data.put("encoder1", e1);
        data.put("encoder2", e2);
        data.put("encoder3", e3);
        data.put("encoder4", e4);
        data.put("current1", c1);
        data.put("current2", c2);
        data.put("current3", c3);
        data.put("current4", c4);
        return data;
    }

}
