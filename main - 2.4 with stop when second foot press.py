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

    def relay_sequence(self, delay_ch2):
        #  CH1 uses static delay; CH2 uses variable delay;
        STATIC_DELAY_CH1 = 2.0  # seconds

        for ch in (1, 2):
            pin = getattr(gpio, f"GPIO_PIN_CH{ch}")

            # Determine delay based on channel
            if ch == 1:
                delay_s = STATIC_DELAY_CH1      # adjustable delay
            else:
                delay_s = delay_ch2   # fixed delay for CH2

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

def debounce(pin, delay_ms=50):
    if pin.value() == 0:  # button pressed
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < delay_ms:
            if pin.value() != 0:
                return False  # bounced back up
        return True
    return False

# ========================
# Initial Values & Display
# ========================

mode       = 0    # 0 = adjust cycles; 1 = adjust time
count      = 6
delay_s    = 3.5  # default relay-on time

update_lcd_count(count)

# ========================
# Main Loop
# ========================

is_running = False  # True while dispensing
cancel = False      # True if user cancels mid-process

def wait_for_button_release(pin):
    while pin.value() == 0:
        time.sleep_ms(10)

while True:

    # Mode switch (only allowed when not running)
    if not is_running and debounce(mode_button):
        mode = 1 - mode
        if mode == 0:
            update_lcd_count(count)
        else:
            update_lcd_time(delay_s)
        wait_for_button_release(mode_button)

    # Increase / Decrease (only allowed when not running)
    if not is_running and debounce(increase_button):
        if mode == 0:
            count += 1
            update_lcd_count(count)
            print("Count →", count)
        else:
            delay_s = round(delay_s + 0.5, 1)
            update_lcd_time(delay_s)
            print("Delay →", delay_s)
        wait_for_button_release(increase_button)

    if not is_running and debounce(decrease_button):
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
        wait_for_button_release(decrease_button)

    # Execute relay sequence
    if not is_running and debounce(relay_button) and count > 0:
        wait_for_button_release(relay_button)

        print(f"Running {count} cycles @ {delay_s}s each")
        total = count
        is_running = True
        cancel = False

        for i in range(total):
            update_lcd_working(total - i, total, delay_s)
            
            # Run both relays with cancel detection
            for ch in (1, 2):
                pin = getattr(gpio, f"GPIO_PIN_CH{ch}")
                delay = delay_s if ch == 2 else 2.0

                print(f"Relay CH{ch} ON for {delay}s")
                pin.value(1)
                relay.flags[ch-1] = True

                start = time.ticks_ms()
                while (time.ticks_diff(time.ticks_ms(), start) < int(delay * 1000)):
                    # Check for cancel while waiting
                    if relay_button.value() == 0:
                        cancel = True
                        break
                    time.sleep_ms(10)

                pin.value(0)
                relay.flags[ch-1] = False
                print(f"Relay CH{ch} OFF")

                if cancel:
                    break  # exit relay loop early

            if cancel:
                break  # exit cycle loop early

        # Handle cancel or completion
        if cancel:
            print("Relay sequence cancelled by user.")
            lcd.clear()
            lcd.putstr("*** CANCELLED ***\n")
            lcd.putstr("*!*!*!*!*!*!*!*!*")
            time.sleep(5)
        else:
            print("Sequence complete.")

        # Restore display
        if mode == 0:
            update_lcd_count(count)
        else:
            update_lcd_time(delay_s)

        is_running = False
        wait_for_button_release(relay_button)

    time.sleep_ms(50)

