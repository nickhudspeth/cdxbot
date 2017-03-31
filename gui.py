#!/usr/bin/env python
import rospy
from std_msgs.msg import String


import kivy
# kivy.require('1.0.6')
from kivy.config import Config
#  Config.set('graphics', 'resizable', 0)  # Disable window resizing
#  Config.set('graphics', 'borderless', 1)  # Disable window bordering
# from kivy.core.window import Window
# Window.size = (500, 300)
#  Config.set('graphics', 'fullscreen', 'auto')  # Set window to match screen res
Config.set('kivy', 'window_icon', './res/cdxicon.png')
Config.set('kivy','exit_on_escape', 0)
from kivy.app import App
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.gridlayout import GridLayout

from time import sleep

def buttonPressCallback(instance):
    print('The button <%s> has been pressed' % instance.text)
    pub.publish('%s' % instance.text)

def shutdownCallback(instance):
    print('The button <%s> has been pressed' % instance.text)
    sd_pub.publish('%s' % instance.text)
    #  sleep(2)
    App.get_running_app().stop()
    rospy.shutdown()

class MyScreen(GridLayout):
    pass
    def __init__(self, **kwargs):
        super(MyScreen, self).__init__(**kwargs)
        self.cols = 2
        self.rows = 2
        self.run_button = Button(text='RUN')
        self.run_button.bind(on_press=buttonPressCallback)
        self.add_widget(self.run_button)
        self.stop_button = Button(text='STOP')
        self.stop_button.bind(on_press=buttonPressCallback)
        self.add_widget(self.stop_button)
        self.shutdown_button = Button(text='SHUTDOWN')
        self.shutdown_button.bind(on_press=shutdownCallback)
        self.add_widget(self.shutdown_button)


class CDXBotGui(App):

    def build(self):
        return MyScreen()


if __name__ == '__main__':
    pub = rospy.Publisher('gui_cmd', String, queue_size=10)
    sd_pub = rospy.Publisher('sd_pub', String, queue_size=10)
    rospy.init_node('gui_node', anonymous=False)
    while not rospy.is_shutdown():
        CDXBotGui().run()
