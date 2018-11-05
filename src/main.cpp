#include "mbed.h"
#include "Messenger.h"
#include "PIN_MAP.h"
#include "helper_functions.h"
#include "Controller.h"

void led_write(std::array<DigitalOut, 4> &LEDs, uint8_t num) {
	LEDs[0] = ((num >> 0) & 1);
	LEDs[1] = ((num >> 1) & 1);
	LEDs[2] = ((num >> 2) & 1);
	LEDs[3] = ((num >> 3) & 1);
}

void bat_watcher(std::array<DigitalOut, 4> &LEDs, AnalogIn &battery_vin) {
	double vbat = battery_vin.read() * (3.3 * 1470 / 470);
	double threshold = (vbat - 6.6) / 1.4;

	if (threshold >= 0.75) led_write(LEDs, 0b1111);
	else if (threshold >= 0.5) led_write(LEDs, 0b0111);
	else if (threshold >= 0.25) led_write(LEDs, 0b0011);
	else led_write(LEDs, 0b0001);
}

void spin(Controller &controller, bool spin_right) {
	if (spin_right) controller.set_target_velocity(-0.25f, 0.25f);
	else controller.set_target_velocity(0.25f, -0.25f);
	wait_ms(200);
	controller.set_target_velocity(0, 0);
	wait_ms(100);
};

int main() {
	std::array<DigitalOut, 4> LEDs = {DigitalOut(LED1), DigitalOut(LED2),
									  DigitalOut(LED3), DigitalOut(LED4)};
	AnalogIn battery_vin(ALL_CELLS);
	bat_watcher(LEDs, battery_vin);

	static Controller controller;
	static Messenger messenger(&controller);

	controller.start_thread();
	messenger.start_thread();

	spin(controller, true);
	spin(controller, false);
	spin(controller, false);
	spin(controller, true);
	controller.timeout.start();

	while (true) {
		bat_watcher(LEDs, battery_vin);
		Thread::wait(200);
//		messenger.send_log(controller.left_wheel.velocity,
//						   controller.right_wheel.velocity);
	}
}
