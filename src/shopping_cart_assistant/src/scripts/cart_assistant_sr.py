#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import speech_recognition as sr

class SpeechRecognitionNode:
    def __init__(self):
        rospy.init_node('speech_recognition_node', anonymous=True)
        self.pub_tts = rospy.Publisher('result', String, queue_size=10)
        self.cart = []
        self.detected_objects = []
        
        self.price_dict = {
            "person": 100.00,
            "remote": 0.50,
            "cell phone": 0.75,
            "milk": 2.50,
            "bread": 1.50
        }

        rospy.Subscriber('input', String, self.object_detection_callback)
        self.recognizer = sr.Recognizer()

    def object_detection_callback(self, data):
        detected_labels = data.data.split(",")
        
        # Store detected objects for later use
        if "No objects detected." in detected_labels:
            rospy.loginfo("No objects detected.")
            self.pub_tts.publish("Please place an object in front of the camera")
        else:
            # rospy.loginfo("Detected objects: %s", detected_labels)
            self.detected_objects = detected_labels  # Update detected objects

    def listen_for_commands(self):
        while not rospy.is_shutdown():
            # Check if there are new detected objects
            if self.detected_objects and not self.detected_objects == []:
                with sr.Microphone() as source:
                    rospy.loginfo(">>> Say something!")
                    audio = self.recognizer.listen(source)
                
                try:
                    result = self.recognizer.recognize_google(audio, language="en-US")
                    rospy.loginfo("SR result: " + result)
                    self.handle_speech_result(result)
                except sr.UnknownValueError:
                    rospy.loginfo("SR could not understand audio")
                    #self.pub_tts.publish("I did not understand that. Please repeat.")
                except sr.RequestError as e:
                    rospy.loginfo("Could not request results from Google Speech Recognition service; {}".format(e))
                    self.pub_tts.publish("Could not request speech recognition service. Please try again later.")
                
                # Clear detected_objects after processing
                self.detected_objects = []
            else:
                rospy.loginfo("Waiting for new detected objects...")
                rospy.sleep(1)  # Wait for 1 second before checking again

    def handle_speech_result(self, result):
        if "price" in result.lower():
            object_info_list = ["{} with price ${}".format(obj, self.price_dict.get(obj, 'unknown')) for obj in self.detected_objects[:5]]
            object_info = ', '.join(object_info_list)
            self.pub_tts.publish("Detected objects: {}".format(object_info))
        elif "repeat" in result.lower():
            object_info_list = ["{} with price ${}".format(obj, self.price_dict.get(obj, 'unknown')) for obj in self.detected_objects[:5]]
            object_info = ', '.join(object_info_list)
            self.pub_tts.publish("Detected objects: {}".format(object_info))
        elif "add to cart" in result.lower():
            for obj in self.detected_objects[:5]:
                self.cart.append(obj)
            self.pub_tts.publish("Items added to cart")
        elif "done" in result.lower():
            self.pub_tts.publish("Your items have been added to the cart. Thank you!")
        else:
            self.pub_tts.publish("I did not understand that. Please repeat.")

    def run(self):
        self.listen_for_commands()  # Start listening for commands

if __name__ == '__main__':
    try:
        sr_node = SpeechRecognitionNode()
        sr_node.run()
    except rospy.ROSInterruptException:
        pass
