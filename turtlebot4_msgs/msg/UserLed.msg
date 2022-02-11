# This message sets the state of the user LEDs
# Blink period is the time in milliseconds during which the ON/OFF cycle occurs
# The duty cycle represents the percentage of the blink period that the LED is ON
# A duty cycle of 1.0 would set the LED to always be ON, whereas a duty cycle of 0.0 is always OFF
# A blink period of 1000ms with a duty cycle of 0.6 will have the LED turn ON for 600ms, 
# then OFF for 400ms

# Available LEDs
uint8 USER_LED_1 = 0
uint8 USER_LED_2 = 1

# Available colors
uint8 COLOR_OFF = 0
uint8 COLOR_GREEN = 1
uint8 COLOR_RED = 2
uint8 COLOR_YELLOW = 3 


# Which available LED to use
uint8 led

# Which color to set the LED to
uint8 color

# Blink period in ms
uint32 blink_period

# Duty cycle (0.0 to 1.0)
float64 duty_cycle