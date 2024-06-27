#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import speech_recognition as sr
from price_list import price_dict

class SpeechRecognitionNode:
    def __init__(self):
        rospy.init_node('speech_recognition_node', anonymous=True)
        self.pub_tts = rospy.Publisher('sr_result', String, queue_size=10)
        self.pub_cart = rospy.Publisher('cart_content', String, queue_size=10)
        self.cart = []
        self.detected_objects = []
        self.repeat_speech = ""
        self.price_dict = price_dict
        self.no_input_timer = None
        self.no_input_timeout = rospy.Duration(10)  # Timeout duration in seconds
        rospy.Subscriber('image_detection_result', String, self.object_detection_callback)
        self.recognizer = sr.Recognizer()

    def object_detection_callback(self, data):
        detected_labels = data.data.split(",")
        
 
        if "No objects detected." in detected_labels:
            rospy.loginfo("No objects detected.")
            self.pub_tts.publish("Please place an object in front of the camera")
        else:
            # rospy.loginfo("Detected objects: %s", detected_labels)
            self.detected_objects = detected_labels  

    def listen_for_commands(self):
        while not rospy.is_shutdown():
            # Check if there are new detected objects
            if self.detected_objects and not self.detected_objects == []:
                with sr.Microphone() as source:
                    rospy.loginfo(">>> Say something!")
                    audio = self.recognizer.listen(source)
                
                if not self.no_input_timer:
                    self.no_input_timer = rospy.Timer(self.no_input_timeout, self.no_input_timeout_callback)

                try:
                    result = self.recognizer.recognize_google(audio, language="en-US")
                    rospy.loginfo("SR result: " + result)
                    self.handle_speech_result(result)
                except sr.UnknownValueError:
                    rospy.loginfo("SR could not understand audio")
                except sr.RequestError as e:
                    rospy.loginfo("Could not request results from Google Speech Recognition service; {}".format(e))
                    self.pub_tts.publish("Could not request speech recognition service. Please try again later.")
                
                # Clear detected_objects after processing
                self.detected_objects = []
            else:
                if not self.no_input_timer:
                    self.no_input_timer = rospy.Timer(self.no_input_timeout, self.no_input_timeout_callback)
                rospy.loginfo("Waiting for new detected objects...")
                rospy.sleep(1)  

    def no_input_timeout_callback(self, event):
        text = "Do you like to place the item in your cart?"
        rospy.loginfo(text)
        handle_speech_result(text)
        self.no_input_timer = None  # Reset the timer
        
    def handle_speech_result(self, result):
        if "price" in result.lower():
            check_input_timer()
            object_info_list = ["{} with price ${}".format(obj, self.price_dict.get(obj, 'unknown')) for obj in self.detected_objects[:5]]
            object_info = ', '.join(object_info_list)
            text = "Detected objects: {}".format(object_info)
            self.repeat_speech = text
            self.pub_tts.publish(text)
        elif "repeat" in result.lower():
            check_input_timer()
            if self.repeat_speech != "":
                self.pub_tts.publish(self.repeat_speech)
            else:
                self.pub_tts.publish("No text to be repeated")
        elif "add to cart" in result.lower():
            check_input_timer()
            for obj in self.detected_objects[:5]:
                if obj in self.price_dict:
                    self.cart.append(obj)
                    cart_content = ','.join(self.cart)
                    self.pub_cart.publish(cart_content)
            self.pub_tts.publish("Items added to cart")
        
        elif "list item" in result.lower():
            check_input_timer()
            if self.cart:
                cart_content = ','.join(self.cart)
                self.pub_cart.publish(cart_content)
                cart_text = "Items in your cart: {}".format(', '.join(self.cart))
                self.repeat_speech = cart_text
                self.pub_tts.publish(cart_text)
            else:
                self.pub_tts.publish("Your cart is empty")
        
        
        elif "total" in result.lower():
            check_input_timer()
            total_price = 0.0
            for item in self.cart:
                total_price += self.price_dict.get(item, 0)

            total_text = "Total price of items in your cart is ${:.2f}".format(total_price)
            self.repeat_speech = total_text
            self.pub_tts.publish(total_text)
           


        elif "done" in result.lower():
            self.pub_tts.publish("Your items have been added to the cart. Thank you!")
        else:
            self.pub_tts.publish("I did not understand that. Please repeat.")

    def check_input_timer(self):
        if self.no_input_timer:
            self.no_input_timer.shutdown()
            self.no_input_timer = None
            
    def run(self):
        self.listen_for_commands()  # Start listening for commands

if __name__ == '__main__':
    try:
        sr_node = SpeechRecognitionNode()
        sr_node.run()
    except rospy.ROSInterruptException:
        pass
