package org.usfirst.frc4904.robot;

import java.util.HashMap;
import java.util.function.DoubleConsumer;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Registers a number on the smart dashboard
 */
public class FunnyNumber {
    public static final boolean ENABLED = true;
    // public static final boolean ENABLED = false;

    private static final HashMap<String, Pair<Double, DoubleConsumer>> numbers = new HashMap<>();
    private static void insertIfEmpty(String name, double defaultValue, DoubleConsumer setter) {
        if (!numbers.containsKey(name)) {
            numbers.put(name, Pair.of(defaultValue, setter)); 
            SmartDashboard.putNumber(name, defaultValue);
        }
    }
    private static double pollDashboard(String name, double val, DoubleConsumer setter) {
        var got = SmartDashboard.getNumber(name, val);
        if (got != val) {
            if (setter != null) setter.accept(got);
            numbers.put(name, Pair.of(got, setter));    // will overwrite previous setter
        }
        return got;
    }
    public static double funnynumber(String name, double defaultValue, DoubleConsumer setter) {
        if (ENABLED) {
            insertIfEmpty(name, defaultValue, setter);
            var n = numbers.get(name);
            return pollDashboard(name, n.getFirst(), n.getSecond());
        } else {
            return defaultValue;
        }
    }
    public static double funnynumber(String name, double defaultValue) {
        return funnynumber(name, defaultValue);
    }
    public static double sdlog(String name, double value) {
        SmartDashboard.putNumber(name, value);
        return value;
    }
}
