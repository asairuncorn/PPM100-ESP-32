from machine import Pin
import time
import ws_gpio

# Create the ws_gpio object which provides access to GPIO pins and functions.
gpio = ws_gpio.GPIO()

# Relay class with a method to run the required sequential activation
class Relay():
    def __init__(self):
        self.Relay_Flag = [False] * 6  # Relay status for 6 channels

    def relay_sequence(self):
        # Activate Relay 1 for 2 seconds
        print("|*** Relay CH1 activated for 2 seconds ***|")
        gpio.GPIO_PIN_CH1.value(1)
        self.Relay_Flag[0] = True
        gpio.Buzzer_PWM(100)
        time.sleep(2)  # Relay 1 active for 2 seconds

        # Release Relay 1
        gpio.GPIO_PIN_CH1.value(0)
        self.Relay_Flag[0] = False
        print("|*** Relay CH1 released ***|")
        gpio.Buzzer_PWM(100)

        # Activate Relay 2 for 3 seconds
        print("|*** Relay CH2 activated for 3 seconds ***|")
        gpio.GPIO_PIN_CH2.value(1)
        self.Relay_Flag[1] = True
        gpio.Buzzer_PWM(100)
        time.sleep(3)  # Relay 2 active for 3 seconds

        # Release Relay 2
        gpio.GPIO_PIN_CH2.value(0)
        self.Relay_Flag[1] = False
        print("|*** Relay CH2 released ***|")
        gpio.Buzzer_PWM(100)

# Create a Relay instance.
relay = Relay()

# Set up the button on IO12.
# It is assumed that the button connects the pin to ground when pressed (active LOW),
# so an internal pull-up resistor is enabled.
button = Pin(12, Pin.IN, Pin.PULL_UP)

def debounce_button(pin):
    """Simple debounce check: if the pin reads LOW, wait 50 ms and check again."""
    if pin.value() == 0:
        time.sleep_ms(50)
        return pin.value() == 0
    return False

# Main loop: Check the button state and trigger the relay sequence when pressed.
while True:
    if debounce_button(button):
        relay.relay_sequence()
        # Wait until the button is released to prevent multiple triggers
        while button.value() == 0:
            time.sleep_ms(10)
    time.sleep_ms(50)
