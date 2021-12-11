#include "car.hpp"

#include <LiquidCrystal.h>
#include <Wire.h>
#include <RingBuf.h>

#define joy_x A1
#define joy_y A0
#define lcd_cols 20
#define max_coord 1023
#define mid_coord 512
#define min_coord 0
/* One motor rotation ~= 185 pulses, tire diameter = 4.3cm
    => C ~= 13.5088484cm => 1 pulse = 0.0730208cm */
#define pulse_cm_ratio 0.0730208
    /* Compass I2C bus address */
#define CMPS14_address 0x60

static auto init_coord = 0, joystick_offset = 0;
static auto x_coord_pos_percent = 0;

// pins declaration
const uint8_t rhs_encoder = 23, lhs_encoder = 24, lhs_encoder_int = 3, rhs_encoder_int = 2;
const uint8_t rs = 37, en = 36, d4 = 35, d5 = 34, d6 = 33, d7 = 32;
const uint8_t m2_pwm = 10, m1_pwm = 9, m2_dir = 8, m1_dir = 7, jtk_btn_pin = 19;
 
volatile uint8_t is_rx_finished = 0;

// Buffer is for commands from ESP.
RingBuf<char, 16> rx_usart_buff;

Car car({ { 4.3, 185 }, m1_pwm, m1_dir, min_coord, max_coord },
    { { 4.3, 185 }, m2_pwm, m2_dir, min_coord, max_coord }, CMPS14_address);
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Update horizontal position of the joystick on display.
void update_lcd(int prev_x_percent, int x_coord_pos_percent)
{
    if (prev_x_percent != x_coord_pos_percent) {
        lcd.setCursor(prev_x_percent, 3);
        lcd.print(" ");
        lcd.setCursor(x_coord_pos_percent, 3);
        lcd.print("|");
    }

    lcd.setCursor(0, 0);
    lcd.print("L:");
    lcd.print(Motor::pulses_to_cm(car.left_motor.pulses));
    lcd.setCursor(lcd_cols / 2, 0);
    lcd.print("R:");
    lcd.print(Motor::pulses_to_cm(car.right_motor.pulses));
}

// Arduino's map has issues. It's simple map implementation.
inline char map_to_percent_range(int val)
{
    if (val > joystick_offset) {
        val += joystick_offset;
    }
    auto x = ceil(
        ((val - min_coord) / static_cast<double>(max_coord - min_coord))
        * 100) / 100.0;
    return (lcd_cols - 1) * x;
}

// Map horizontal joystick position. Update car motors with the new values.
inline void jtk_to_car_motors(int val, Car* car)
{
    // Correction for bad initial joystick values.
    if (val > joystick_offset) {
        val += joystick_offset;
    }
    auto new_speed = (min(val + 1, max_coord) - mid_coord) / 2;
    car->update_speed(new_speed, new_speed);
}

// Joystick button pressed -> motors turned off.
void joy_btn_ISR()
{
    car.toggle_motors_power();
}

// Update pulses of the left wheel.
void left_wheel_ISR()
{
    if (car.left_motor.get_direction() == HIGH) {
        ++car.left_motor.pulses;
    } else {
        --car.left_motor.pulses;
    }
}

// Update pulses of the right wheel.
void right_wheel_ISR()
{
    if (car.right_motor.get_direction() == HIGH) {
        ++car.right_motor.pulses;
    } else {
        --car.right_motor.pulses;
    }
}

// Once transmission finished, extract ring buffer chars one by one and return string.
String reconstruct_command()
{
    // 3ch (command name) + 1ch (':') + 3ch (value) + 1ch ('\n)
    String command = String();
    char push_data;
    while (rx_usart_buff.lockedPop(push_data)) {
        // Add buffer char to command string
        command += push_data;
    }
    return command;
}

// RX interrupt to catch all transmitted chars.
ISR(USART0_RX_vect)
{
    // Once we face '\n', change transmission flag.
    if (UDR0 == '\n') { is_rx_finished = 1; }
    if (!rx_usart_buff.isFull()) { rx_usart_buff.push(UDR0); }
}

void setup()
{
    Wire.begin(1);

    // Sort of calibration.
    init_coord = analogRead(joy_x);
    x_coord_pos_percent = init_coord, y_coord_percent = init_coord;
    joystick_offset = mid_coord - init_coord;

    // Baud rate 9600bps.
    UBRR0 = 103;
    // Set usage of 8-bit char sizes.
    UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
    // Turn on transmission, reception, recieve interrupts.
    UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    // Enable global interrupt.
    sei();

    pinMode(lhs_encoder, INPUT);
    pinMode(rhs_encoder, INPUT);

    pinMode(m2_pwm, INPUT);
    pinMode(m1_pwm, INPUT);
    pinMode(m2_dir, INPUT);
    pinMode(m1_dir, INPUT);
    pinMode(jtk_btn_pin, INPUT_PULLUP);

    lcd.begin(lcd_cols, 4);
    lcd.clear();
    lcd.setCursor(0, 2);
    lcd.print("-100%");
    lcd.setCursor(lcd_cols / 2 - 1, 2);
    lcd.print("0%");
    lcd.setCursor(lcd_cols - 4, 2);
    lcd.print("100%");

    // All our interrupts will work on FALLING edge. Just simpler.
    attachInterrupt(digitalPinToInterrupt(rhs_encoder_int), &right_wheel_ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(lhs_encoder_int), &left_wheel_ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(jtk_btn_pin), &joy_btn_ISR, FALLING);
}

void loop()
{
    // Poll command from UART.
    if (is_rx_finished) {
        auto command = reconstruct_command();
        // All command names should be 3ch long.
        auto command_name = command.substring(1, 4);
        // Take values of the commands.
        auto command_value = static_cast<int>(command.substring(5).toInt());

        if (command_name == "dst") {
            car.drive_n_cm(command_value);
        } else if (command_name == "rot") {
            car.turn_degree(command_value);
        } else if (command_name == "vel") {
            car.update_speed(command_value, command_value);
        }

        // Clear transmission flag.
        is_rx_finished = 0;
    }

    auto prev_x_percent = x_coord_pos_percent;
    auto current_x_pos = analogRead(joy_x);
    x_coord_pos_percent = map_to_percent_range(current_x_pos);
    update_lcd(prev_x_percent, x_coord_pos_percent);
    car.update_compass_angle();

    // Can block car velocity command from ESP if motors enabled.
    if (car.is_powered_on()) {
        jtk_to_car_motors(current_x_pos, &car);
    }
}
