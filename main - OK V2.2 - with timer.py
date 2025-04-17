from machine import SoftI2C, Pin
import time
import ws_gpio
from lcd_api import LcdApi
from i2c_lcd import I2cLcd

# ========================
# Setup the 2004A LCD Display
# ========================

I2C_ADDR  = 0x27          # Try 0x27 (or 0x3F)
NUM_ROWS  = 4             # 4 rows for a 2004A display
NUM_COLS  = 20            # 20 columns

i2c = SoftI2C(scl=Pin(3), sda=Pin(39), freq=400000)
lcd = I2cLcd(i2c, I2C_ADDR, NUM_ROWS, NUM_COLS)

def update_lcd_count(cnt):
    lcd.clear()
    lcd.putstr("=> Mode: Cycles\n\n")
    lcd.putstr("  Cycles: {:>3}".format(cnt))

def update_lcd_time(delay_s):
    lcd.clear()
    lcd.putstr("=> Mode: Time\n\n")
    lcd.putstr("  Delay: {:.1f}s".format(delay_s))

def update_lcd_working(cycles_left, total, delay_s):
    lcd.clear()
    lcd.putstr("*** DISPENSING ***\n")
    lcd.putstr("Cycles: {}/{}\n".format(total-cycles_left+1, total))
    lcd.putstr("Delay: {:.1f}s".format(delay_s))

# ========================
# Relay and Button Setup
# ========================

gpio = ws_gpio.GPIO()

class Relay():
    def __init__(self):
        self.flags = [False]*6

    def relay_sequence(self, delay_s):
        # repeat for channels 1 and 2 as example
        for ch in (1,2):
            pin = getattr(gpio, f"GPIO_PIN_CH{ch}")
            print(f"Relay CH{ch} ON for {delay_s}s")
            pin.value(1)
            self.flags[ch-1] = True
            time.sleep(delay_s)
            pin.value(0)
            self.flags[ch-1] = False
            print(f"Relay CH{ch} OFF")

relay = Relay()

# Buttons
mode_button     = Pin(7,  Pin.IN, Pin.PULL_UP)   # Switch between modes
relay_button    = Pin(8,  Pin.IN, Pin.PULL_UP)   # Start sequence
decrease_button = Pin(9,  Pin.IN, Pin.PULL_UP)   # “–”
increase_button = Pin(10, Pin.IN, Pin.PULL_UP)   # “+”

def debounce(pin):
    if pin.value() == 0:
        time.sleep_ms(50)
        if pin.value() == 0:
            return True
    return False

# ========================
# Initial Values & Display
# ========================

mode       = 0    # 0 = adjust cycles; 1 = adjust time
count      = 0
delay_s    = 2.5  # default relay-on time

update_lcd_count(count)

# ========================
# Main Loop
# ========================

while True:

    # Mode switch
    if debounce(mode_button):
        mode = 1 - mode
        if mode == 0:
            update_lcd_count(count)
        else:
            update_lcd_time(delay_s)
        # wait for release
        while mode_button.value() == 0:
            time.sleep_ms(10)

    # Increase / Decrease in current mode
    if debounce(increase_button):
        if mode == 0:
            count += 1
            update_lcd_count(count)
            print("Count →", count)
        else:
            delay_s = round(delay_s + 0.5, 1)
            update_lcd_time(delay_s)
            print("Delay →", delay_s)
        while increase_button.value() == 0:
            time.sleep_ms(10)

    if debounce(decrease_button):
        if mode == 0:
            if count > 0:
                count -= 1
            update_lcd_count(count)
            print("Count ←", count)
        else:
            if delay_s > 0.5:
                delay_s = round(delay_s - 0.5, 1)
            update_lcd_time(delay_s)
            print("Delay ←", delay_s)
        while decrease_button.value() == 0:
            time.sleep_ms(10)

    # Execute relay sequence
    if debounce(relay_button) and count > 0:
        print(f"Running {count} cycles @ {delay_s}s each")
        total = count
        for i in range(total):
            update_lcd_working(total-(i), total, delay_s)
            relay.relay_sequence(delay_s)
        # restore display
        if mode == 0:
            update_lcd_count(count)
        else:
            update_lcd_time(delay_s)
        while relay_button.value() == 0:
            time.sleep_ms(10)

    time.sleep_ms(50)
