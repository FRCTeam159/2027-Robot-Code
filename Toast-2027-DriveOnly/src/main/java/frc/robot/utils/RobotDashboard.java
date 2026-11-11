package frc.robot.utils;

import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;

import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.BooleanSubscriber;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringPublisher;

/**
 * Small NetworkTables-backed replacement for the removed SmartDashboard API.
 * Entries live under the "Robot" table; chooser-style controls can be supplied by publishing
 * values to the same topics from a dashboard client.
 */
public final class RobotDashboard {
    private static final NetworkTable TABLE = NetworkTableInstance.getDefault().getTable("Robot");
    private static final ConcurrentMap<String, BooleanPublisher> BOOLEAN_PUBLISHERS = new ConcurrentHashMap<>();
    private static final ConcurrentMap<String, DoublePublisher> DOUBLE_PUBLISHERS = new ConcurrentHashMap<>();
    private static final ConcurrentMap<String, StringPublisher> STRING_PUBLISHERS = new ConcurrentHashMap<>();
    private static final ConcurrentMap<String, BooleanSubscriber> BOOLEAN_SUBSCRIBERS = new ConcurrentHashMap<>();
    private static final ConcurrentMap<String, DoubleSubscriber> DOUBLE_SUBSCRIBERS = new ConcurrentHashMap<>();

    private RobotDashboard() {}

    public static void putBoolean(String key, boolean value) {
        BOOLEAN_PUBLISHERS.computeIfAbsent(key, k -> TABLE.getBooleanTopic(k).publish()).set(value);
    }

    public static boolean getBoolean(String key, boolean defaultValue) {
        return BOOLEAN_SUBSCRIBERS.computeIfAbsent(key,
                k -> TABLE.getBooleanTopic(k).subscribe(defaultValue)).get();
    }

    public static void putNumber(String key, double value) {
        DOUBLE_PUBLISHERS.computeIfAbsent(key, k -> TABLE.getDoubleTopic(k).publish()).set(value);
    }

    public static double getNumber(String key, double defaultValue) {
        return DOUBLE_SUBSCRIBERS.computeIfAbsent(key,
                k -> TABLE.getDoubleTopic(k).subscribe(defaultValue)).get();
    }

    public static void putString(String key, String value) {
        STRING_PUBLISHERS.computeIfAbsent(key, k -> TABLE.getStringTopic(k).publish()).set(value);
    }
}
