import sqlite3
import pandas as pd
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# ── CONFIG ──────────────────────────────────────────────────────────────────
# Use the internal Docker paths, starting at /home/racecar_ws/
BAG_FILE = r"/home/racecar/racecar_ws/src/visual_servoing/visual_servoing/visual_servoing/parking_rosbags/outrange_real_3/outrange_real_3_0.db3"

# Save the CSV right next to the bag file
OUTPUT_CSV = r"/home/racecar/racecar_ws/src/visual_servoing/visual_servoing/visual_servoing/parking_rosbags/outrange_real_3/parking_error.csv"

TOPIC_NAME = "/parking_error"
# ────────────────────────────────────────────────────────────────────────────
def extract_bag_to_csv(bag_path, topic_name, output_csv):
    print(f"Opening ROS 2 bag: {bag_path}")

    try:
        # Connect to the SQLite3 database ROS 2 uses for bags
        conn = sqlite3.connect(bag_path)
        cursor = conn.cursor()
    except sqlite3.OperationalError:
        print(f"Error: Could not find or open the database at {bag_path}")
        return

    # 1. Look up the topic ID and message type
    cursor.execute("SELECT id, type FROM topics WHERE name = ?", (topic_name,))
    topic_info = cursor.fetchone()

    if topic_info is None:
        print(f"Error: Topic '{topic_name}' not found in the bag.")
        print("Available topics:")
        cursor.execute("SELECT name FROM topics")
        for row in cursor.fetchall():
            print(f"  - {row[0]}")
        return

    topic_id, topic_type = topic_info
    print(f"Found topic '{topic_name}' of type '{topic_type}'")

    # 2. Dynamically load the message class (e.g., vs_msgs/msg/ParkingError)
    try:
        msg_class = get_message(topic_type)
    except ModuleNotFoundError:
        print(f"\nError: Could not find the message definition for {topic_type}.")
        print("Make sure you have sourced your ROS 2 workspace: 'source install/setup.bash'")
        return

    # 3. Pull all messages for this topic
    cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = ?", (topic_id,))
    rows = cursor.fetchall()

    if not rows:
        print(f"No messages found on topic '{topic_name}'.")
        return

    print(f"Extracting and deserializing {len(rows)} messages...")

    csv_data = []
    for timestamp, data in rows:
        # Deserialize the raw CDR formatted binary data into the Python object
        msg = deserialize_message(data, msg_class)

        # Build the dictionary for this row
        row_dict = {
            'time': timestamp,
            'x_error': getattr(msg, 'x_error', 0.0),
            'y_error': getattr(msg, 'y_error', 0.0),
            'distance_error': getattr(msg, 'distance_error', 0.0)
        }
        csv_data.append(row_dict)

    # 4. Save to CSV
    df = pd.DataFrame(csv_data)
    df.to_csv(output_csv, index=False)
    print(f"\nSuccess! Saved to: {output_csv}")

if __name__ == "__main__":
    extract_bag_to_csv(BAG_FILE, TOPIC_NAME, OUTPUT_CSV)
