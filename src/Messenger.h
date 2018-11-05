#ifndef MESSENGER_H_
#define MESSENGER_H_

#include "mbed.h"
#include "XBeeLib.h"
#include <string>
#include <array>
#include "helper_functions.h"
#include "Controller.h"

template<int size>
struct MsgData {
	std::array<float, size> data;
	bool is_valid;

	/**	@brief Operator overload, simplifies access to data array
	 *	@param i Data array index
	 *	@return Float value stoered in data */
	inline float &operator[](int i) {
		return data[i];
	}
};

class Messenger {

	private:
	XBeeLib::XBee802 xbee;
	Thread xbee_thread;
	uint16_t xbee_addr;

	Controller *controller;

	void xbee_thread_callback();

	/**	@brief Sends battery voltage. Example: "B7.53" */
	void send_battery();

	/**	@brief Decodes received message and executes command
		 *	@param msg Message containing command to be executed */
	void decode_msg(const std::string &msg);

	/**	@brief Breaks msg using ';' as delimiter, converts each substring to float and returns array with the result
	 *	@tparam size Expected number of substrings, also size of returned array
	 *	@param msg Input string, to be divided in substrings
	 *	@param first_char_pos Function ignores all chars before first_char_position
	 *	@return Struct containing the resulting float array and flag is_valid,
	 *	set to false if number of substrings is smaller than expected */
	template<int size>
	MsgData<size> get_values(const std::string &msg, unsigned int first_char_pos);

	public:
	/**	@brief Constructor
	 *	@param id A unique ID assigned to each robot
	 *	@param robot Pointer to Robot, used to set constants and start controllers
	 *	@param this_xbee XBee802 object, used for sending and receiving messages
	 *	@param sensors_ptr Pointer to SensorFusion, used to set new data for an EKF vision update*/
	explicit Messenger(Controller *controller_ptr);

	/**	@brief Sends message to another xbee
	 *	@param msg Message to be sent
	 *	@param addr 16-bit address of the receiving xbee */
	void send_msg(const std::string &msg, uint16_t addr = 0x35D0);

	void start_thread();

//	Sends csv logs
	template <typename ...T>
	void send_log(T ...data) {
		std::string msg;
		append(msg, data...);
		send_msg(msg);
	}
};

#endif /* MESSENGER_H_ */
