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
# BTN X GPIO26 (37) (38) GPIO20  JOY N
#          GND (39) (40) GPIO21  JOY S
#
# MQTT INTERFACE:
# mosquitto_sub -h 192.168.147.1    -t joy_box/signal   <--- bridged on router
# mosquitto_sub -h 192.168.147.176  -t joy_box/signal   <--- joy_box mqtt server
#
# press x y a b on held --> shutdown
#
# MQTT EXAMPLES:
# {"module_type": "JoyBoxServer", "contact": "joy_s", "action": "pressed", "timestamp": "2021-05-26T19:44:06.544778"}
# {"module_type": "JoyBoxServer", "contact": "btn_a", "action": "pressed", "timestamp": "2021-05-26T19:45:09.871762"}
# {"module_type": "JoyBoxServer", "contact": "btn_a", "action": "held", "timestamp": "2021-05-26T19:45:10.881939"}
# {"module_type": "JoyBoxServer", "contact": "btn_a", "action": "released", "timestamp": "2021-05-26T19:45:11.424959"}
# {"module_type": "JoyBoxServer", "contact": "system", "action": "poweroff", "timestamp": "2021-05-26T19:45:11.424959"}
#

import json
import logging
from subprocess import check_call
from datetime import datetime
from gpiozero import Button
import paho.mqtt.client as mqtt

TOPIC_SIGNAL = "joy_box/signal"
TOPIC_SETTINGS = "joy_box/settings"
BT = None


class JoyBoxServer:
    def __init__(self, mqtt_broker='localhost', mqtt_port=1883, level=logging.INFO):
        logging.getLogger().setLevel(level=level)
        self.module_type = 'JoyBoxServer'
        self.joy_n = Button(20, bounce_time=BT)
        self.joy_n.when_pressed = self.joy_n_when_pressed
        self.joy_n.when_released = self.joy_n_when_released
        self.joy_n.when_held = self.joy_n_when_held
        self.joy_s = Button(21, bounce_time=BT)
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
        #
        self.shutdown_request = False
        self.btn_a_held = False
        self.btn_b_held = False
        self.btn_x_held = False
        self.btn_y_held = False
        #
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
        j = {'module_type': self.module_type, 'contact': contact, 'action': action, 'timestamp': datetime.now().isoformat()}
        self.mqtt.publish(topic=TOPIC_SIGNAL, payload=json.dumps(j))

    def joy_n_when_pressed(self):
        logging.debug("{}: {}".format(self.module_type, 'joy_n_when_pressed'))
        self.mqtt_publish("joy_n", "pressed")

    def joy_n_when_released(self):
        logging.debug("{}: {}".format(self.module_type, 'joy_n_when_released'))
        self.mqtt_publish("joy_n", "released")

    def joy_n_when_held(self):
        logging.debug("{}: {}".format(self.module_type, 'joy_n_when_held'))
        self.mqtt_publish("joy_n", "held")

    def joy_s_when_pressed(self):
        logging.debug("{}: {}".format(self.module_type, 'joy_s_when_pressed'))
        self.mqtt_publish("joy_s", "pressed")

    def joy_s_when_released(self):
        logging.debug("{}: {}".format(self.module_type, 'joy_s_when_released'))
        self.mqtt_publish("joy_s", "released")

    def joy_s_when_held(self):
        logging.debug("{}: {}".format(self.module_type, 'joy_s_when_held'))
        self.mqtt_publish("joy_s", "held")

    def joy_e_when_pressed(self):
        logging.debug("{}: {}".format(self.module_type, 'joy_e_when_pressed'))
        self.mqtt_publish("joy_e", "pressed")

    def joy_e_when_released(self):
        logging.debug("{}: {}".format(self.module_type, 'joy_e_when_released'))
        self.mqtt_publish("joy_e", "released")

    def joy_e_when_held(self):
        logging.debug("{}: {}".format(self.module_type, 'joy_e_when_held'))
        self.mqtt_publish("joy_e", "held")

    def joy_w_when_pressed(self):
        logging.debug("{}: {}".format(self.module_type, 'joy_w_when_pressed'))
        self.mqtt_publish("joy_w", "pressed")

    def joy_w_when_released(self):
        logging.debug("{}: {}".format(self.module_type, 'joy_w_when_released'))
        self.mqtt_publish("joy_w", "released")

    def joy_w_when_held(self):
        logging.debug("{}: {}".format(self.module_type, 'joy_w_when_held'))
        self.mqtt_publish("joy_w", "held")

    def btn_x_when_pressed(self):
        logging.debug("{}: {}".format(self.module_type, 'btn_x_when_pressed'))
        self.mqtt_publish("btn_x", "pressed")

    def btn_x_when_released(self):
        logging.debug("{}: {}".format(self.module_type, 'btn_x_when_released'))
        self.mqtt_publish("btn_x", "released")
        self.btn_x_held = False

    def btn_x_when_held(self):
        logging.debug("{}: {}".format(self.module_type, 'btn_x_when_held'))
        self.mqtt_publish("btn_x", "held")
        self.btn_x_held = True
        self.check_shutdown_request()

    def btn_y_when_pressed(self):
        logging.debug("{}: {}".format(self.module_type, 'btn_y_when_pressed'))
        self.mqtt_publish("btn_y", "pressed")

    def btn_y_when_released(self):
        logging.debug("{}: {}".format(self.module_type, 'btn_y_when_released'))
        self.mqtt_publish("btn_y", "released")
        self.btn_y_held = False

    def btn_y_when_held(self):
        logging.debug("{}: {}".format(self.module_type, 'btn_y_when_held'))
        self.mqtt_publish("btn_y", "held")
        self.btn_y_held = True
        self.check_shutdown_request()

    def btn_a_when_pressed(self):
        logging.debug("{}: {}".format(self.module_type, 'btn_a_when_pressed'))
        self.mqtt_publish("btn_a", "pressed")

    def btn_a_when_released(self):
        logging.debug("{}: {}".format(self.module_type, 'btn_a_when_released'))
        self.mqtt_publish("btn_a", "released")
        self.btn_a_held = False

    def btn_a_when_held(self):
        logging.debug("{}: {}".format(self.module_type, 'btn_a_when_held'))
        self.mqtt_publish("btn_a", "held")
        self.btn_a_held = True
        self.check_shutdown_request()

    def btn_b_when_pressed(self):
        logging.debug("{}: {}".format(self.module_type, 'btn_b_when_pressed'))
        self.mqtt_publish("btn_b", "pressed")

    def btn_b_when_released(self):
        logging.debug("{}: {}".format(self.module_type, 'btn_b_when_released'))
        self.mqtt_publish("btn_b", "released")
        self.btn_b_held = False

    def btn_b_when_held(self):
        logging.debug("{}: {}".format(self.module_type, 'btn_b_when_held'))
        self.mqtt_publish("btn_b", "held")
        self.btn_b_held = True
        self.check_shutdown_request()

    def check_shutdown_request(self):
        if self.btn_a_held and self.btn_b_held and self.btn_x_held and self.btn_y_held:
            self.mqtt_publish("system", "poweroff")
            check_call(['sudo', 'poweroff'])


if __name__ == '__main__':
    joy_box = JoyBoxServer()
