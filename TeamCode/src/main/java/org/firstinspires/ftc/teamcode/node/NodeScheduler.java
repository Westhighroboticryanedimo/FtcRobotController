package org.firstinspires.ftc.teamcode.node;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class NodeScheduler {
    // TODO: add logging as a special node because it will write the entire data map
    // TODO: special case if a node does not subscribe to anything
    // INFO: why node-based programming? it's deterministic (more predictable robot actions and
    // allows for replays w/ logging), highly modular, easy for the programmer to use, functional-style
    // style points for being ROS-inspired

    List<Node> nodes;
    HashMap<String, Object> data;
    HashMap<Node, List<String>> subscriptions;

    public NodeScheduler(Node... nodeList) {
        nodes = Arrays.asList(nodeList);
        data.put("default", null);
    }

    public void init() {
        for (Node n : nodes) {
            n.init();
        }
        for (Node n : nodes) {
            subscriptions.put(n, n.getSubscriptions());
        }
    }
    public void end() {
        for (Node n : nodes) {
            n.end();
        }
    }
    public void update() {
        // update and exchange data
        for (Node n : nodes) {
            data.putAll(n.publish());
        }
        for (Node n : nodes) {
            HashMap<String, Object> message = new HashMap<>();
            for (String s : subscriptions.get(n)) {
                message.put(s, data.get(s));
            }
            n.receive(message);
        }
        // loop through node actions
        for (Node n : nodes) {
            n.loop();
        }
    }
}
