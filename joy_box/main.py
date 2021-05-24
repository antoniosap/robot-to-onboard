#!/usr/bin/env python3
#
# 23.5.2021
#
# joy_box ---> MQTT
#
#  --    GPIO5 (29) (30) GND
# BTN B  GPIO6 (31) (32) GPIO12  JOY W
# BTN A GPIO13 (33) (34) GND
# BTN Y GPIO19 (35) (36) GPIO16  JOY E
# BTN X GPIO26 (37) (38) GPIO20  JOY S
#          GND (39) (40) GPIO21  JOY N

import json
import logging
from gpiozero import Button
import paho.mqtt.client as mqtt

TOPIC_SIGNAL = "joy_box/signal"
TOPIC_SETTINGS = "joy_box/settings"
BT = 0.05


class JoyBoxServer:
    def __init__(self, mqtt_broker='localhost', mqtt_port=1883, level=logging.INFO):
        logging.getLogger().setLevel(level=level)
        self.module_type = 'JoyBoxServer'
        self.joy_n = Button(21, bounce_time=BT)
        self.joy_n.when_pressed = self.joy_n_when_pressed
        self.joy_n.when_released = self.joy_n_when_released
        self.joy_n.when_held = self.joy_n_when_held
        self.joy_s = Button(20, bounce_time=BT)
        self.joy_s.when_pressed = self.joy_s_when_pressed
        self.joy_s.when_released = self.joy_s_when_released
        self.joy_s.when_held = self.joy_s_when_held
        self.joy_e = Button(16, bounce_time=BT)
        self.joy_e.when_pressed = self.joy_e_when_pressed
        self.joy_e.when_released = self.joy_e_when_released
        self.joy_e.when_held = self.joy_e_when_held
        self.joy_w = Button(12, bounce_time=BT)
        self.joy_w.when_pressed = self.joy_w_when_pressed
        self.joy_w.when_released = self.joy_w_when_released
        self.joy_w.when_held = self.joy_w_when_held
        self.btn_x = Button(26, bounce_time=BT)
        self.btn_x.when_pressed = self.btn_x_when_pressed
        self.btn_x.when_released = self.btn_x_when_released
        self.btn_x.when_held = self.btn_x_when_held
        self.btn_y = Button(19, bounce_time=BT)
        self.btn_y.when_pressed = self.btn_y_when_pressed
        self.btn_y.when_released = self.btn_y_when_released
        self.btn_y.when_held = self.btn_y_when_held
        self.btn_a = Button(13, bounce_time=BT)
        self.btn_a.when_pressed = self.btn_a_when_pressed
        self.btn_a.when_released = self.btn_a_when_released
        self.btn_a.when_held = self.btn_a_when_held
        self.btn_b = Button(6, bounce_time=BT)
        self.btn_b.when_pressed = self.btn_b_when_pressed
        self.btn_b.when_released = self.btn_b_when_released
        self.btn_b.when_held = self.btn_b_when_held
        self.mqtt_broker = mqtt_broker
        self.mqtt_port = mqtt_port
        self.mqtt_connected = False
        self.mqtt = mqtt.Client(protocol=mqtt.MQTTv311)
        self.mqtt.on_connect = self.mqtt_on_connect
        self.mqtt.on_disconnect = self.mqtt_on_disconnect
        self.mqtt.on_message = self.mqtt_on_message
        self.mqtt.connect(mqtt_broker, mqtt_port, keepalive=60)
        self.mqtt.loop_forever()  # blocking

    def mqtt_on_connect(self, client, userdata, flags, result_code):
        if result_code != mqtt.CONNACK_ACCEPTED:
            logging.error("{}: unable to connect to the MQTT broker: {}".format(self.module_type, mqtt.connack_string(result_code)))
            return

        self.mqtt_connected = True
        logging.info("{}: connected to MQTT broker {}:{} ({})".format(self.module_type, self.mqtt_broker, self.mqtt_port, result_code))
        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        client.subscribe(TOPIC_SIGNAL)

    def mqtt_on_disconnect(self, client, userdata, result_code):
        """Disconnected callback."""
        self.mqtt_connected = False
        logging.warning("{}: disconnected from MQTT server {}:{} ({})".format(self.module_type, self.mqtt_broker, self.mqtt_port, result_code))

    def mqtt_on_message(self, client, userdata, msg):
        """Message received callback."""
        logging.debug("{}: topic {} value {}".format(self.module_type, msg.topic, str(msg.payload)))
        value = json.loads(msg.payload)
        if msg.topic == TOPIC_SETTINGS:
            pass

    def mqtt_publish(self, contact, action):
        j = {'module_type': self.module_type, 'contact': contact, 'action': action}
        self.mqtt.publish(topic=TOPIC_SIGNAL, payload=json.dumps(j))

    def joy_n_when_pressed(self):
        logging.debug("{}: {}".format(self.module_type, 'joy_n_when_pressed'))
        self.mqtt_publish("joy_n", "when_pressed")

    def joy_n_when_released(self):
        logging.debug("{}: {}".format(self.module_type, 'joy_n_when_released'))
        self.mqtt_publish("joy_n", "when_released")

    def joy_n_when_held(self):
        logging.debug("{}: {}".format(self.module_type, 'joy_n_when_held'))
        self.mqtt_publish("joy_n", "when_held")

    def joy_s_when_pressed(self):
        logging.debug("{}: {}".format(self.module_type, 'joy_s_when_pressed'))
        self.mqtt_publish("joy_s", "when_pressed")

    def joy_s_when_released(self):
        logging.debug("{}: {}".format(self.module_type, 'joy_s_when_released'))
        self.mqtt_publish("joy_s", "when_released")

    def joy_s_when_held(self):
        logging.debug("{}: {}".format(self.module_type, 'joy_s_when_held'))
        self.mqtt_publish("joy_s", "when_held")

    def joy_e_when_pressed(self):
        logging.debug("{}: {}".format(self.module_type, 'joy_e_when_pressed'))
        self.mqtt_publish("joy_e", "when_pressed")

    def joy_e_when_released(self):
        logging.debug("{}: {}".format(self.module_type, 'joy_e_when_released'))
        self.mqtt_publish("joy_e", "when_released")

    def joy_e_when_held(self):
        logging.debug("{}: {}".format(self.module_type, 'joy_e_when_held'))
        self.mqtt_publish("joy_e", "when_held")

    def joy_w_when_pressed(self):
        logging.debug("{}: {}".format(self.module_type, 'joy_w_when_pressed'))
        self.mqtt_publish("joy_w", "when_pressed")

    def joy_w_when_released(self):
        logging.debug("{}: {}".format(self.module_type, 'joy_w_when_released'))
        self.mqtt_publish("joy_w", "when_released")

    def joy_w_when_held(self):
        logging.debug("{}: {}".format(self.module_type, 'joy_w_when_held'))
        self.mqtt_publish("joy_w", "when_held")

    def btn_x_when_pressed(self):
        logging.debug("{}: {}".format(self.module_type, 'btn_x_when_pressed'))
        self.mqtt_publish("btn_x", "when_pressed")

    def btn_x_when_released(self):
        logging.debug("{}: {}".format(self.module_type, 'btn_x_when_released'))
        self.mqtt_publish("btn_x", "when_released")

    def btn_x_when_held(self):
        logging.debug("{}: {}".format(self.module_type, 'btn_x_when_held'))
        self.mqtt_publish("btn_x", "when_held")

    def btn_y_when_pressed(self):
        logging.debug("{}: {}".format(self.module_type, 'btn_y_when_pressed'))
        self.mqtt_publish("btn_y", "when_pressed")

    def btn_y_when_released(self):
        logging.debug("{}: {}".format(self.module_type, 'btn_y_when_released'))
        self.mqtt_publish("btn_y", "when_released")

    def btn_y_when_held(self):
        logging.debug("{}: {}".format(self.module_type, 'btn_y_when_held'))
        self.mqtt_publish("btn_y", "when_held")

    def btn_a_when_pressed(self):
        logging.debug("{}: {}".format(self.module_type, 'btn_a_when_pressed'))
        self.mqtt_publish("btn_a", "when_pressed")

    def btn_a_when_released(self):
        logging.debug("{}: {}".format(self.module_type, 'btn_a_when_released'))
        self.mqtt_publish("btn_a", "when_released")

    def btn_a_when_held(self):
        logging.debug("{}: {}".format(self.module_type, 'btn_a_when_held'))
        self.mqtt_publish("btn_a", "when_held")

    def btn_b_when_pressed(self):
        logging.debug("{}: {}".format(self.module_type, 'btn_b_when_pressed'))
        self.mqtt_publish("btn_b", "when_pressed")

    def btn_b_when_released(self):
        logging.debug("{}: {}".format(self.module_type, 'btn_b_when_released'))
        self.mqtt_publish("btn_b", "when_released")

    def btn_b_when_held(self):
        logging.debug("{}: {}".format(self.module_type, 'btn_b_when_held'))
        self.mqtt_publish("btn_b", "when_held")


if __name__ == '__main__':
    joy_box = JoyBoxServer(level=logging.DEBUG)
