package org.firstinspires.ftc.teamcode.node;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public class NodeScheduler {
    // TODO: add logging as a special node because it will write the entire data map, also timer
    // TODO: add special case if a node does not subscribe to anything
    // INFO: why node-based programming? it's deterministic (more predictable robot actions and
    // allows for replays w/ logging), highly modular, easy for the programmer to use, functional-style
    // also atomic (kind of) and safe, the only thing a node can do is publish and receive data
    // and whatever it does in its loop
    // style points for being ROS-inspired

    // ARCH: a robot has multiple subsystems or services that are running concurrently: running the
    // drivetrain, controlling a lift, visual localization, filtering data, calculating odo, etc.
    // A node can do something when it starts, do something when it ends, does something on each
    // iteration, and publishes and subscribes to data provided by other nodes. Every loop, every
    // node will publish data to the node scheduler, which will hold every piece of data and pass to
    // each node data from their respective subscriptions.

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
        // publish and exchange data
        publish();
        exchange();
        // loop through node actions
        for (Node n : nodes) {
            n.loop();
        }
    }
    // for anything external (not a node) that needs data, e.g. opmode telemetry
    public HashMap<String, Object> extract(String... queries) {
        HashMap<String, Object> d = new HashMap<>();
        for (String q : queries) {
            d.put(q, data.get(q));
        }
        return d;
    }

    private void publish() {
        for (Node n : nodes) {
            data.putAll(n.publish());
        }
    }
    private void exchange() {
        for (Node n : nodes) {
            HashMap<String, Object> message = new HashMap<>();
            for (String s : subscriptions.get(n)) {
                message.put(s, data.get(s));
            }
            n.receive(message);
        }
    }
}
