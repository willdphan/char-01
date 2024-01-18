#!/usr/bin/env python3

import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.feature_selection import SelectKBest
from sklearn.preprocessing import LabelEncoder
import joblib
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class Predictor(Node):
    def __init__(self):
        super().__init__('predictor')
        self.publisher_ = self.create_publisher(String, 'prediction', 10)
        # Predictor node subscribes to the lidar topic.
        # Whenever a message is published on this topic,
        # the listener_callback method is called with message as an argument
        self.subscription = self.create_subscription(
            LaserScan,
            'lidar',
            self.listener_callback,
            10)

        self.subscription  # prevent unused variable warning
        # Load the trained model
        self.clf = joblib.load(
            '/media/psf/Developer/Robotics/char-01/ros2_pkg/data/rf_model/random_forest_model.pkl')
        # Load the feature selector
        self.k_best = joblib.load(
            '/media/psf/Developer/Robotics/char-01/ros2_pkg/data/k_best/k_best.pkl')
        # Load the label encoder
        self.label_encoder = joblib.load(
            '/media/psf/Developer/Robotics/char-01/ros2_pkg/data/label_encoder/label_encoder.pkl')

    # the received data is preprocessed and
    # then passed to the model to make a prediction.
    def listener_callback(self, msg):
        print("listener_callback called")
        # Convert the LaserScan data to a numpy array
        data = np.array(msg.ranges)
        # Select a subset of the range readings
        data = data[:240]
        # Replace infinities with a large finite number
        data[data == np.inf] = 1e10
        # Preprocess the data
        data = self.preprocess_data(data)
        # Make a prediction
        prediction = self.clf.predict(data)
        print("Made prediction:", prediction)
        # Convert the prediction to a string
        prediction_str = ' '.join(map(str, prediction))
        # Create a String message and set its data field
        prediction_msg = String()
        prediction_msg.data = prediction_str
        # Publish the prediction
        self.publisher_.publish(prediction_msg)

    # Preprocess data here
    def preprocess_data(self, data):
        # This will depend on how you preprocessed your data when training the model
        # For example, you might need to select the best features like this:
        data = self.k_best.transform(data.reshape(1, -1))
        return data


def main(args=None):
    rclpy.init(args=args)

    predictor = Predictor()

    rclpy.spin(predictor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    predictor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
