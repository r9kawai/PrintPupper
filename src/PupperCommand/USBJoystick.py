import time
import select
import struct
import threading
from collections import OrderedDict

"""
where js_event is defined as
	struct js_event {
		__u32 time;     /* event timestamp in milliseconds */
		__s16 value;    /* value */
		__u8 type;      /* event type */
		__u8 number;    /* axis/button number */
	};
"""
EVENT_FORMAT = "IhBB";
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

class USBJoystick():
    def __init__(self, devpath):
        print('USBJoystick start...')
        self.fd = open(devpath, "rb")
        self.v = OrderedDict()
        self.v["left_analog_x"] = 0
        self.v["left_analog_y"] = 0
        self.v["right_analog_x"] = 0
        self.v["right_analog_y"] = 0
        self.v["button_r1"] = False
        self.v["button_l1"] = False
        self.v["button_square"] = False
        self.v["button_cross"] = False
        self.v["button_circle"] = False
        self.v["button_triangle"] = False
        self.v["dpad_right"] = 0
        self.v["dpad_left"] = 0
        self.v["dpad_up"] = 0
        self.v["dpad_down"] = 0
        self.reader_exit = threading.Event()
        self.reader_thread = threading.Thread(target=self.joystick_reader)
        self.reader_thread.start()
        self.active = True

    def __del__(self):
        self.close()
        return

    def close(self):
        if self.active:
            self.fd.close()
            self.reader_exit.set()
            self.reader_thread.join(timeout=1)
            self.active = False
            print('USBJoystick...done')
        return

    def get_input(self):
        return self.v

    def led_color(self, red=0, green=0, blue=0):
        return

    def joystick_reader(self):
        while not self.reader_exit.is_set():
            try:
                js_data = self.fd.read(EVENT_SIZE)
                if len(js_data) == EVENT_SIZE:
                    (js_time, js_value, js_type, js_number) = struct.unpack(EVENT_FORMAT, js_data)
                else:
                    continue
            except:
                continue

            """
            js_dbg = '{0:08x} '.format(js_time)
            js_dbg += '{: >6} '.format(js_value)
            js_dbg += '{0:02x} '.format(js_type)
            js_dbg += '{0:02x} : '.format(js_number)
            print(js_dbg)
            """

            if js_type & 0x7f == JS_EVENT_BUTTON:
                vkey = False
                if js_number == 0:
                    vkey = 'button_square'
                if js_number == 1:
                    vkey = 'button_triangle'
                if js_number == 2:
                    vkey = 'button_cross'
                if js_number == 3:
                    vkey = 'button_circle'
                if js_number == 4:
                    vkey = 'button_l1'
                if js_number == 5:
                    vkey = 'button_r1'
                if vkey != False:
                    if js_value == 1:
                        self.v[vkey] = True
                    else:
                        self.v[vkey] = False

            if js_type & 0x7f == JS_EVENT_AXIS:
                if (js_number >= 0) and (js_number <= 3):
                    vkey = False
                    if js_number == 0:
                        vkey = 'left_analog_x'
                    if js_number == 1:
                        vkey = 'left_analog_y'
                    if js_number == 2:
                        vkey = 'right_analog_y'
                    if js_number == 3:
                        vkey = 'right_analog_x'
                    analog_value = round(float(js_value / 32767), 2)
                    if vkey:
                        self.v[vkey] = analog_value

                if js_number == 4:
                    if js_value > 0:
                        self.v['dpad_right'] = 1
                        self.v['dpad_left'] = 0
                    if js_value < 0:
                        self.v['dpad_right'] = 0
                        self.v['dpad_left'] = 1
                    if js_value == 0:
                        self.v['dpad_right'] = 0
                        self.v['dpad_left'] = 0

                if js_number == 5:
                    if js_value > 0:
                        self.v['dpad_up'] = 0
                        self.v['dpad_down'] = 1
                    if js_value < 0:
                        self.v['dpad_up'] = 1
                        self.v['dpad_down'] = 0
                    if js_value == 0:
                        self.v['dpad_up'] = 0
                        self.v['dpad_down'] = 0


if __name__ == '__main__':
    joystick = USBJoystick('/dev/input/js0')
    for i in range(100):
        values = joystick.get_input()
        print(values)
        time.sleep(0.1)
    joystick.led_color(255, 0, 0)
    joystick.close()


