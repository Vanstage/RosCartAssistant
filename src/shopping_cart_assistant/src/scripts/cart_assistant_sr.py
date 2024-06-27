#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import speech_recognition as sr
from price_list import price_dict

class SpeechRecognitionNode:
    def __init__(self):
        rospy.init_node('speech_recognition_node', anonymous=True)  # Initialize ROS node for speech recognition
        # Publisher for speech recognition results and cart content updates
        self.pub_tts = rospy.Publisher('sr_result', String, queue_size=10)  
        self.pub_cart = rospy.Publisher('cart_content', String, queue_size=10) 
        # Initialize empty cart list and list for detected objects
        self.cart = []
        self.detected_objects = [] 
        self.repeat_speech = ""  
        self.price_dict = price_dict  # Load price dictionary from external module
        self.no_input_timer = None  # Timer for no input timeout
        self.no_input_timeout = rospy.Duration(10)  # Timeout duration in seconds
        # Subscribe to object detection results
        rospy.Subscriber('image_detection_result', String, self.object_detection_callback)  
        # Initialize speech recognizer
        self.recognizer = sr.Recognizer()  

    def object_detection_callback(self, data):
        detected_labels = data.data.split(",") 
        
        # Handle no objects detected scenario
        if "No objects detected." in detected_labels:
            rospy.loginfo("No objects detected.")
            # Publish TTS message if no object detected
            self.pub_tts.publish("Please place an object in front of the camera")  
        else:
            # Start no input timer if not already started, this timer run until user input something 
            # If no input in 10 seconds, no_input_timeout_callback will be called
            if not self.no_input_timer:
                self.no_input_timer = rospy.Timer(self.no_input_timeout, self.no_input_timeout_callback)
            # Update detected objects list
            self.detected_objects = detected_labels  

    def listen_for_commands(self):
        while not rospy.is_shutdown():
            # Check if there are new detected objects to process
            if self.detected_objects and not self.detected_objects == []:
                with sr.Microphone() as source:
                    rospy.loginfo(">>> Say something!")
                    audio = self.recognizer.listen(source)  # Listen for audio input
                
                try:
                    result = self.recognizer.recognize_google(audio, language="en-US")  # Perform speech recognition
                    rospy.loginfo("SR result: " + result)  
                    self.handle_speech_result(result)  # Process recognized speech
                except sr.UnknownValueError:
                    # Handle unrecognized audio
                    rospy.loginfo("SR could not understand audio")  
                except sr.RequestError as e:
                    # Handle service request error
                    rospy.loginfo("Could not request results from Google Speech Recognition service; {}".format(e))
                    self.pub_tts.publish("Could not request speech recognition service. Please try again later.") 
                # Clear detected objects after processing
                self.detected_objects = []  
            else:
                if not self.no_input_timer:
                    self.no_input_timer = rospy.Timer(self.no_input_timeout, self.no_input_timeout_callback)
                rospy.loginfo("Waiting for new detected objects...")
                rospy.sleep(1)  # Sleep briefly while waiting for new objects

    def no_input_timeout_callback(self, event):
        # Prompt for user action
        text = "Do you like to place the item in your cart?"  
        rospy.loginfo(text)  
        self.handle_speech_result(text)  # Process prompt action
        self.no_input_timer = None  # Reset the timer

    def handle_speech_result(self, result):
        if "price" in result.lower():
            # Check and reset input timer
            self.check_input_timer()  
            # Generate object info list with prices
            object_info_list = ["{} with price ${}".format(obj, self.price_dict.get(obj, 'unknown')) for obj in self.detected_objects[:5]]
            object_info = ', '.join(object_info_list)  
            text = "Detected objects: {}".format(object_info)  
            # Store speech for potential repetition
            self.repeat_speech = text  
            # Publish speech response
            self.pub_tts.publish(text) 
        elif "repeat" in result.lower():
            # Check and reset input timer
            self.check_input_timer()  
            if self.repeat_speech != "":
                # Publish repeated speech
                self.pub_tts.publish(self.repeat_speech)  
            else:
                # Handle no text to repeat
                self.pub_tts.publish("No text to be repeated")  
        elif "add to cart" in result.lower():
            self.check_input_timer()  # Check and reset input timer
            # Add detected objects to cart
            for obj in self.detected_objects[:5]:
                if obj in self.price_dict:
                    self.cart.append(obj)  # Add object to cart list
                    cart_content = ','.join(self.cart)  # Format cart content as string
                    self.pub_cart.publish(cart_content)  # Publish updated cart content
            self.pub_tts.publish("Items added to cart")  # Publish confirmation message
        elif "list item" in result.lower():
            self.check_input_timer()  # Check and reset input timer
            # List items currently in cart
            if self.cart:
                cart_content = ','.join(self.cart)  # Format cart content as string
                self.pub_cart.publish(cart_content)  # Publish current cart content
                cart_text = "Items in your cart: {}".format(', '.join(self.cart))  # Format cart response
                self.repeat_speech = cart_text  # Store speech for potential repetition
                self.pub_tts.publish(cart_text)  # Publish cart content
            else:
                self.pub_tts.publish("Your cart is empty")  # Handle empty cart scenario
        elif "total" in result.lower():
            self.check_input_timer()  # Check and reset input timer
            # Calculate total price of items in cart
            total_price = 0.0
            for item in self.cart:
                total_price += self.price_dict.get(item, 0)  # Sum prices from price dictionary

            total_text = "Total price of items in your cart is ${:.2f}".format(total_price)  # Format total price response
            self.repeat_speech = total_text  # Store speech for potential repetition
            self.pub_tts.publish(total_text)  # Publish total price
        elif "done" in result.lower():
            self.pub_tts.publish("Your items have been added to the cart. Thank you!")  # Publish completion message
        else:
            self.pub_tts.publish("I did not understand that. Please repeat.")  # Handle unrecognized speech

    def check_input_timer(self):
        if self.no_input_timer:
            self.no_input_timer.shutdown()  # Shutdown existing input timer
            self.no_input_timer = None  # Reset input timer

    def run(self):
        self.listen_for_commands()  # Start listening for speech commands

if __name__ == '__main__':
    try:
        sr_node = SpeechRecognitionNode()
        sr_node.run()  # Run the speech recognition node
    except rospy.ROSInterruptException:
        pass  # Handle ROS interrupt exception gracefully
