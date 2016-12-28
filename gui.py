#!/usr/bin/env python
import rospy
from std_msgs.msg import String


import kivy
# kivy.require('1.0.6')
from kivy.config import Config
Config.set('graphics', 'resizable', 0)
from kivy.core.window import Window
Window.size = (500, 300)
from kivy.app import App
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.gridlayout import GridLayout


def buttonPressCallback(instance):
    print('The button <%s> has been pressed' % instance.text)
    pub.publish('%s' % instance.text)


class MyScreen(GridLayout):

    def __init__(self, **kwargs):
        super(MyScreen, self).__init__(**kwargs)
        self.cols = 2
        self.run_button = Button(text='RUN')
        self.run_button.bind(on_press=buttonPressCallback)
        self.add_widget(self.run_button)
        self.stop_button = Button(text='STOP')
        self.stop_button.bind(on_press=buttonPressCallback)
        self.add_widget(self.stop_button)


class CDXBotGui(App):

    def build(self):
        return MyScreen()


if __name__ == '__main__':
    pub = rospy.Publisher('gui_cmd', String, queue_size=10)
    rospy.init_node('gui_node', anonymous=False)
    while not rospy.is_shutdown():
        CDXBotGui().run()
