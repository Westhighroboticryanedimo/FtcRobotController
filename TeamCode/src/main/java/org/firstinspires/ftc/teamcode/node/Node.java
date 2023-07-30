package org.firstinspires.ftc.teamcode.node;

import java.util.HashMap;
import java.util.List;

public class Node {
    // append to this list w/ subscriptions in child class
    protected List<String> subscriptions = List.of("default");
    protected HashMap<String, Object> data = new HashMap<String, Object>();
    // override in child
    public void init() { }
    // override in child
    public void end() { }
    // override in child
    public void loop() { }
    public List<String> getSubscriptions() { return subscriptions; }
    // override in child
    public HashMap<String, Object> publish() { return new HashMap<String, Object>(); }
    public void receive(HashMap<String, Object> d) { data = d; }
}