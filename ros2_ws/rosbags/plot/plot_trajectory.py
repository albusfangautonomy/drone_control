#!/usr/bin/env python3
import argparse
import numpy as np
import matplotlib.pyplot as plt

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def read_series(bag_path, topic):
    storage = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter = ConverterOptions("", "")
    reader = SequentialReader()
    reader.open(storage, converter)

    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in topics:
        raise RuntimeError(f"Topic {topic} not in bag, available: {list(topics.keys())}")

    typ = get_message(topics[topic])
    ts, xs, ys, zs = [], [], [], []
    while reader.has_next():
        tname, raw, _ = reader.read_next()
        if tname != topic:
            continue
        msg = deserialize_message(raw, typ)
        if hasattr(msg, "timestamp"):  # PX4 msgs
            t = msg.timestamp * 1e-6
            if hasattr(msg, "position"):
                x, y, z = msg.position[0], msg.position[1], msg.position[2]
                if np.isnan(x) or np.isnan(y):
                    continue

            else:
                x, y, z = msg.x, msg.y, msg.z
        else:  # nav_msgs/Odometry etc.
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p = msg.pose.pose.position
            x, y, z = p.x, p.y, p.z
        ts.append(t); xs.append(x); ys.append(y); zs.append(z)

    return np.array(ts), np.array(xs), np.array(ys), np.array(zs)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--odom", required=True)
    ap.add_argument("--setpoint", required=True)
    args = ap.parse_args()

    t1, x1, y1, z1 = read_series(args.bag, args.odom)
    t2, x2, y2, z2 = read_series(args.bag, args.setpoint)

    # simple plots
    plt.figure(figsize=(6,6))
    plt.plot(x1, y1, label="actual")
    plt.plot(x2, y2, "--", label="setpoint")
    plt.axis("equal"); plt.xlabel("X"); plt.ylabel("Y"); plt.legend(); plt.title("XY trajectory")
    plt.show()

    plt.figure()
    plt.plot(t1, z1, label="actual z")
    plt.plot(t2, z2, "--", label="setpoint z")
    plt.xlabel("time [s]"); plt.ylabel("Z"); plt.legend(); plt.title("Altitude vs time")
    plt.show()

if __name__ == "__main__":
    main()
