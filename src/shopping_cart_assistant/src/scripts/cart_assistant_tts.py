#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from gtts import gTTS
import os
import time

# Global variables to keep track of the cart state and last detected object
cart = []
last_object = None
object_detected = False

def say(text):
    tts = gTTS(text, lang="en-US")
    tts.save("/tmp/speech.mp3")
    os.system("mpg321 /tmp/speech.mp3")
    os.remove("/tmp/speech.mp3")

def callback(data):
    global cart, last_object, object_detected
    rospy.loginfo("Input: %s", data.data)
    text = data.data.lower()

    if "add to cart" in text:
        if last_object:
            cart.append(last_object)
            say("Added to cart")
            object_detected = False  # Reset for next object detection
        else:
            say("No object detected to add to cart")
    elif "repeat" in text:
        if last_object:
            say(f"The object is {last_object['name']} and costs {last_object['price']}")
        else:
            say("No object detected")
    elif "price" in text and last_object:
        say(f"The price is {last_object['price']}")
    elif "please place an object" in text:
        say("Please place an object in front of the camera")
    elif "cart is empty" in text:
        say("The cart is empty")
    elif text == "i did not understand that." and object_detected:
        say("Do you want to add this object to the cart?")
    elif object_detected:
        say("Do you want to add this object to the cart?")
    else:
        # Handle object detection and description
        if "object detected" in text:
            # Mock object detection
            last_object = {"name": "Sample Object", "price": "$10"}
            say(f"Detected {last_object['name']} priced at {last_object['price']}")
            object_detected = True
        else:
            say("I did not understand that.")

def googletts():
    rospy.init_node('googletts', anonymous=True)

    rospy.Subscriber("result", String, callback)
    
    # Initial prompt for the user to place an object
    say("Please place an object in front of the camera")

    rospy.spin()

if __name__ == '__main__':
    googletts()
