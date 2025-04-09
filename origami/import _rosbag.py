import rosbag2_py
import csv

# Set up storage and converter options
storage = rosbag2_py.StorageOptions(uri='condition3_0.db3', storage_id='sqlite3')  # Include .db3 extension
converter = rosbag2_py.ConverterOptions('', '')  # Empty converter options for default behavior

# Create a SequentialReader and open the bag file with both options
reader = rosbag2_py.SequentialReader()
reader.open(storage, converter)

# Open CSV file for writing
with open('output.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile, escapechar='\\', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    writer.writerow(['String Data'])  # Write header

    # Iterate through messages in the rosbag2
    while reader.has_next():
        (topic, msg, timestamp) = reader.read_next()
        if topic == '/micro_ros_arduino_node_wifipublisher':  # Use the correct topic name
            # Decode byte data to string directly
            string_data = msg.decode('utf-8')  # Decode bytes to string using UTF-8 encoding
            
            # Write the string, escaping commas and special characters
            writer.writerow([string_data])  # Write the decoded string to CSV file

print("Data saved successfully!")


