# File: TopicStatus.msg
# Name of the topic (e.g., /camera/image_raw)
string topic_name

# Message type string (e.g., sensor_msgs/msg/Image)
string topic_type

# True if the topic was actively publishing messages during the check, False otherwise
bool is_publishing