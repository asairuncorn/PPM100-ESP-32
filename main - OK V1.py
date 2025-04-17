from machine import SoftI2C, Pin
import time
import ws_gpio
from lcd_api import LcdApi
from i2c_lcd import I2cLcd

# ========================
# Setup the 2004A LCD Display
# ========================

I2C_ADDR = 0x27          # Try 0x27 (or 0x3F if that doesn’t work)
NUM_ROWS = 4             # 4 rows for a 2004A display
NUM_COLS = 20            # 20 columns per row

i2c = SoftI2C(scl=Pin(3), sda=Pin(39), freq=400000)
lcd = I2cLcd(i2c, I2C_ADDR, NUM_ROWS, NUM_COLS)

def update_lcd_count(cnt):
    lcd.clear()
    lcd.putstr("Select No of Cycles\n")
    lcd.putstr("\n")
    lcd.putstr("  Cycles Count: {}\n".format(cnt))

def update_lcd_working(cycles_left, total):

    lcd.clear()
        
    lcd.putstr(" ***  WORKING  ***\n\n")
    lcd.putstr("  Cycles left: {}\n".format(cycles_left))
    

update_lcd_count(0)

# ========================
# Relay and Button Code
# ========================

gpio = ws_gpio.GPIO()

class Relay():
    def __init__(self):
        self.Relay_Flag = [False] * 6

    def relay_sequence(self):
        print("|*** Relay CH1 activated for 4 seconds ***|")
        gpio.GPIO_PIN_CH1.value(1)
        self.Relay_Flag[0] = True
        time.sleep(2.5)
        gpio.GPIO_PIN_CH1.value(0)
        self.Relay_Flag[0] = False
        print("|*** Relay CH1 released ***|")
        
        print("|*** Relay CH2 activated for 4 seconds ***|")
        gpio.GPIO_PIN_CH2.value(1)
        self.Relay_Flag[1] = True
        time.sleep(2.5)
        gpio.GPIO_PIN_CH2.value(0)
        self.Relay_Flag[1] = False
        print("|*** Relay CH2 released ***|")

relay = Relay()

# Buttons:
relay_button = Pin(8, Pin.IN, Pin.PULL_UP)      # Relay trigger (Active LOW)
increase_button = Pin(10, Pin.IN, Pin.PULL_UP)     # Increase count (Active LOW)
decrease_button = Pin(9, Pin.IN, Pin.PULL_UP)     # Decrease count (Active LOW)

def debounce_button(pin):
    if pin.value() == 0:
        time.sleep_ms(50)
        return pin.value() == 0
    return False

count = 0

while True:
    if debounce_button(increase_button):
        count += 1
        update_lcd_count(count)
        print("Count increased:", count)
        while increase_button.value() == 0:
            time.sleep_ms(10)

    if debounce_button(decrease_button):
        if count > 0:
            count -= 1
        update_lcd_count(count)
        print("Count decreased:", count)
        while decrease_button.value() == 0:
            time.sleep_ms(10)

    if debounce_button(relay_button):
        print("Executing relay sequence {} times".format(count))
        total = count  # Save the original count for calculations.
        for i in range(count):
            cycles_left = total - i
            update_lcd_working(cycles_left, total)
            relay.relay_sequence()
        update_lcd_count(count)
        while relay_button.value() == 0:
            time.sleep_ms(10)
 
    time.sleep_ms(50)
