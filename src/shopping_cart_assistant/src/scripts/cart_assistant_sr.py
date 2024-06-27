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
        # Load price dictionary from external module
        self.price_dict = price_dict 
        # Timer for no input timeout
        self.no_input_timer = None 
        # Timeout duration in seconds
        self.no_input_timeout = rospy.Duration(30) 
        # Subscribe to object detection results
        rospy.Subscriber('image_detection_result', String, self.object_detection_callback)  
        # Initialize speech recognizer
        self.recognizer = sr.Recognizer() 
        self.end_process = False 

    def object_detection_callback(self, data):
        detected_labels = data.data.split(",") 
        
        # Handle no objects detected scenario
        if "No objects detected." in detected_labels:
            rospy.loginfo("No objects detected.")
            # Publish TTS message if no object detected
            self.pub_tts.publish("Please place an object in front of the camera")
        elif "Thank you for using me" in detected_labels:
            if not self.end_process:
                self.handle_speech_result("total")
                self.handle_speech_result("done")
                self.end_process = True


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
                rospy.sleep(1) 

    def no_input_timeout_callback(self, event):
        text = "Do you like to place the item in your cart?"  
        rospy.loginfo(text)
        self.repeat_speech = text
        self.pub_tts.publish(text)
        self.check_input_timer()

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
                    cart_content = ','.join(self.cart)  
                    # Publish updated cart content
                    self.pub_cart.publish(cart_content)  
            # Publish confirmation message
            self.pub_tts.publish("Items added to cart")  
        elif "list item" in result.lower():
            self.check_input_timer()  # Check and reset input timer
            # List items currently in cart
            if self.cart:
                cart_content = ','.join(self.cart) 
                # Publish current cart content
                self.pub_cart.publish(cart_content)  
                cart_text = "Items in your cart: {}".format(', '.join(self.cart)) 
                self.repeat_speech = cart_text
                # Publish cart content
                self.pub_tts.publish(cart_text)  
            else:
                # Handle empty cart scenario
                self.pub_tts.publish("Your cart is empty") 
        elif "total" in result.lower():
            self.check_input_timer() 
            # Calculate total price of items in cart
            total_price = 0.0
            for item in self.cart:
                # Sum prices from price dictionary
                total_price += self.price_dict.get(item, 0) 


            total_text = "Total price of items in your cart is ${:.2f}".format(total_price)
            self.pub_tts.publish(total_text)
            if total_price == 0:
                total_text = "Your cart is empty"
                self.repeat_speech = total_text 
             
            # Publish total price
            self.pub_tts.publish(total_text)  
        elif "done" in result.lower():
            # Publish completion message
            self.pub_tts.publish("Thank you for using me!")  
        else:
            # Handle unrecognized speech
            self.pub_tts.publish("I did not understand that. Please repeat.") 

    def check_input_timer(self):
        if self.no_input_timer:
            # Shutdown existing input timer if exist
            self.no_input_timer.shutdown()  
            # Reset input timer 
            self.no_input_timer = None  

    def run(self):
        # Start listening for speech commands
        self.listen_for_commands()  

if __name__ == '__main__':
    try:
        sr_node = SpeechRecognitionNode()
        sr_node.run()  # Run the speech recognition node
    except rospy.ROSInterruptException:
        pass  
