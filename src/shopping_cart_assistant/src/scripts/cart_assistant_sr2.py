#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import speech_recognition as sr

def keyword_listener(recognizer, microphone):
    #Continuously listen for the keyword "add to cart".
    with microphone as source:
        recognizer.adjust_for_ambient_noise(source)
        print(">>> Listening for keyword 'add to cart'")
        while not rospy.is_shutdown():
            try:
                audio = recognizer.listen(source, timeout=5)
                keyword_result = recognizer.recognize_google(audio, language="en-US")
                if "add to cart" in keyword_result.lower():
                    print(">>> Keyword 'add to cart' detected.")
                    return True
            except sr.UnknownValueError:
                continue
            except sr.WaitTimeoutError:
                continue

def detailed_speech_recognition(recognizer, microphone):
    #Perform detailed speech recognition after keyword is detected.
    with microphone as source:
        print(">>> Say something!")
        audio = recognizer.record(source, duration=5)
        
    try:
        result = recognizer.recognize_google(audio, language="en-US")
        print("SR result: " + result)
        return result
    except sr.UnknownValueError:
        print("SR could not understand audio")
        return "I did not understand that."
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))
        return "Request to Google Speech Recognition failed."

def googlesr():
    rospy.init_node('googlesr', anonymous=True)
    pub = rospy.Publisher('result', String, queue_size=10)
    
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()

    while not rospy.is_shutdown():
        if keyword_listener(recognizer, microphone):
            result = detailed_speech_recognition(recognizer, microphone)
            pub.publish(result)

if __name__ == '__main__':
    try:
        googlesr()
    except rospy.ROSInterruptException:
        pass
