#!/usr/bin/env python

"""  """

from sense_hat import SenseHat


sense = SenseHat()
sense.set_rotation(90)
sense.show_message("Hello world!", text_colour=[255, 255, 0])

sense.set_imu_config(True, True, True)
orientation_rad = sense.get_orientation_radians()
print(sense.orientation_radians)

orientation = sense.get_orientation_degrees()
print(orientation)

orientation = sense.get_orientation()
print(orientation)

raw = sense.get_compass_raw()
print(raw)

raw = sense.get_gyroscope_raw()
print(raw)

raw = sense.get_accelerometer_raw()
print(raw)

humidity = sense.get_humidity()
temp = sense.get_temperature()

temp = sense.get_temperature_from_pressure()

pressure = sense.get_pressure()



