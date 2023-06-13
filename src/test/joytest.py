import struct
from collections import OrderedDict

infile_path = "/dev/input/js2"

#struct js_event {
#	__u32 time;	/* event timestamp in milliseconds */
#	__s16 value;	/* value */
#	__u8 type;	/* event type */
#	__u8 number;	/* axis/button number */
#};

# EVENT_FORMAT = "llHHI"; # long, long, unsigned short, unsigned short, unsigned int
EVENT_FORMAT = "IHBB";
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)

jdict = OrderedDict()
jdict['left_analog_x'] = 0
jdict['left_analog_y'] = 0
jdict['right_analog_x'] = 0
jdict['right_analog_y'] = 0
jdict['l2_analog'] = 0
jdict['r2_analog'] = 0
jdict['dpad_up'] = False
jdict['dpad_down'] = False
jdict['dpad_left'] = False
jdict['dpad_right'] = False
jdict['button_cross'] = False
jdict['button_circle'] = False
jdict['button_square'] = False
jdict['button_triangle'] = False
jdict['button_l1'] = False
jdict['button_l2'] = False
jdict['button_l3'] = False
jdict['button_r1'] = False
jdict['button_r2'] = False
jdict['button_r3'] = False
jdict['button_share'] = False
jdict['button_options'] = False
jdict['button_trackpad'] = False
jdict['button_ps'] = False
jdict['motion_y'] = 0
jdict['motion_x'] = 0
jdict['motion_z'] = 0
jdict['orientation_roll'] = 0
jdict['orientation_yaw'] = 0
jdict['orientation_pitch'] = 0
jdict['trackpad_touch0_id'] = 0
jdict['trackpad_touch0_active'] = False
jdict['trackpad_touch0_x'] = 0
jdict['trackpad_touch0_y'] = 0
jdict['trackpad_touch1_id'] = 0
jdict['trackpad_touch1_active'] = False
jdict['trackpad_touch1_x'] = 0
jdict['trackpad_touch1_y'] = 0
jdict['timestamp'] = 0
jdict['battery'] = 100
jdict['plug_usb'] = False
jdict['plug_audio'] = False
jdict['plug_mic'] = False

with open(infile_path, "rb") as file:
    event = True
    while event:
        event = file.read(EVENT_SIZE)
        (jtime, jval, jtype, jbutn) = struct.unpack(EVENT_FORMAT, event)
#       print(jtype, jbutn, jbutn);

        if jtype == 1:
            if jbutn == 4:
                if jval == 0:
                    jdict['button_l1'] = False
                else:
                    jdict['button_l1'] = True

        if jtype == 1:
            if jbutn == 5:
                if jval == 0:
                    jdict['button_r1'] = False
                else:
                    jdict['button_r1'] = True

        print(jdict)
