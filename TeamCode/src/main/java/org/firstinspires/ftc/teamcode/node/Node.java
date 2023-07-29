package org.firstinspires.ftc.teamcode.node;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public class Node {
    protected List<String> subscriptions = List.of("default");
    protected HashMap<String, Object> data = new HashMap<String, Object>();
    public void init() { }
    public void end() { }
    public void loop() { }
    public List<String> getSubscriptions() { return subscriptions; }
    public HashMap<String, Object> publish() { return new HashMap<String, Object>(); }
    public void receive(HashMap<String, Object> d) { data = d; }
}