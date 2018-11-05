#include <string>
#include "Messenger.h"
#include "PIN_MAP.h"
#include "ConfigFile.h"
#include "helper_functions.h"

#define PI 3.1415926f
using std::string;

void Messenger::send_msg(const string &msg, uint16_t addr) {
	XBeeLib::RemoteXBee802 remoteDevice = XBeeLib::RemoteXBee802(addr);
	xbee.send_data(remoteDevice, (const uint8_t *) msg.c_str(), (uint16_t) msg.size(), true);
}

void Messenger::send_battery() {
	AnalogIn vin_all_cells(ALL_CELLS);
	float vbat = vin_all_cells.read() * (3.3f * 1470.0f / 470.0f);
	float vbat_round = std::round(vbat * 1000) / 1000;
	string msg_bat = "B" + std::to_string(vbat_round);
	send_msg(msg_bat);
}

template<int size>
MsgData<size> Messenger::get_values(const string &msg, unsigned int first_char_pos) {
	std::array<float, size> values{};
	unsigned int pos_atual = first_char_pos;
	for (int i = 0; i < size; ++i) {
		size_t delim_pos = msg.find(';', pos_atual);
		if (delim_pos == string::npos && i != size - 1) return {values, false};
		values[i] = std::stof(msg.substr(pos_atual, delim_pos - pos_atual));
		pos_atual = delim_pos + 1;
	}
	return {values, true};
}

void Messenger::decode_msg(const string &msg) {
	switch (msg[0]) {
		case 'B':
			send_battery();
			return;
		case 'P': {
			MsgData<3> values = get_values<3>(msg, 0);
			if (values.is_valid) {
				controller->set_pid_constants(values[0],
											  values[1], values[2]);
			}
			return;
		} default: {
			MsgData<2> values = get_values<2>(msg, 0);
			if (values.is_valid) {
				float right_vel = values[0];
				float left_vel = values[1];
				controller->set_target_velocity(left_vel, right_vel);
			}
		}
	}
}

void Messenger::start_thread() {
	xbee_thread.start(callback(this, &Messenger::xbee_thread_callback));
}

void Messenger::xbee_thread_callback() {
	while (true) {
		Thread::signal_wait(CONTINUE_SIGNAL);
		Thread::signal_clr(CONTINUE_SIGNAL);
		xbee.process_rx_frames();
	}
}

Messenger::Messenger(Controller *controller_ptr)
		: xbee(RADIO_TX, RADIO_RX, RADIO_RESET, NC, NC, 115200) {

	setlocale(LC_ALL, "C");

	controller = controller_ptr;

	{
		ConfigFile configs("/local/config.txt");
		xbee_addr = configs.get_xbee_addr();
	}

	static Messenger *messenger_ptr = this;

	xbee.set_on_complete_callback([]() {
		messenger_ptr->xbee_thread.signal_set(CONTINUE_SIGNAL);
	});

	xbee.register_receive_cb([](const XBeeLib::RemoteXBee802 &remote,
								bool broadcast, const uint8_t *data, uint16_t len) {
		if (len != 0) {
			messenger_ptr->decode_msg(string((const char *) data, len));
		}
	});

	XBeeLib::RadioStatus const radioStatus = xbee.init();
	MBED_ASSERT(radioStatus == XBeeLib::Success);
	xbee.set_network_address(xbee_addr);
}

