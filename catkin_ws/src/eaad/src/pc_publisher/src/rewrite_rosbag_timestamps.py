import rosbag
import sys

# if arguments are less than 3, print usage and exit
if ( len (sys.argv) < 3 or sys.argv[1] == "-h" or sys.argv[1] == "--help" ):
    print("Usage: python rewrite_rosbag_timestamps.py input.bag output.bag")
    sys.exit(0)

input_bag = sys.argv[1]
output_bag = sys.argv[2]

if ( input_bag.split(".")[-1] != "bag" or output_bag.split(".")[-1] != "bag" ):
    print("Error: input and output files must be .bag files")
    sys.exit(0)

# If input bag does not exist, exit
try:
    rosbag.Bag(input_bag)
except:
    print("Error: input bag does not exist")
    sys.exit(0)

with rosbag.Bag( output_bag, 'w') as outbag:
    print ("Rewriting bag .....")
    for topic, msg, t in rosbag.Bag( input_bag ).read_messages():
        # This also replaces tf timestamps under the assumption 
        # that all transforms in the message share the same timestamp
        if topic == "/tf" and msg.transforms:
            outbag.write(topic, msg, msg.transforms[0].header.stamp)
        else:
            outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
