package org.firstinspires.ftc.teamcode.squirmy;

import org.firstinspires.ftc.teamcode.node.Node;

import java.util.HashMap;
import java.util.List;

public class CurrentWatchdogNode extends Node {
    boolean danger = false;

    @Override
    public void init() {
        subscriptions.addAll(List.of("current1", "current2", "current3", "current4"));
    }

    @Override
    public void loop() {
        danger = ((int) data.get("current1") + (int) data.get("current2") + (int) data.get("current3") + (int) data.get("current4")) > 16;
    }

    @Override
    public HashMap<String, Object> publish() {
        HashMap<String, Object> message = new HashMap<String, Object>();
        message.put("current_danger", danger);
        return message;
    }

}