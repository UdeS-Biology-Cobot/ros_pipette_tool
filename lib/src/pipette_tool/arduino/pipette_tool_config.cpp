/*
 * pipette_tool_config.cpp
 *
 *  Created on: Nov 19, 2018
 *      Author: biobot
 */

#if defined(ARDUINO)

#include <robotic_tool/pipette_tool/arduino/pipette_tool_config.h>
#include <stdlib.h>     /* strtof */
#include <errno.h>

PipetteToolConfig::PipetteToolConfig(Serial_* port, uint32_t baudrate, PipetteTool* pt)
: port(port),
  baudrate(baudrate),
  pt(pt)
{
	port->begin(baudrate);
}


PipetteToolConfig::State PipetteToolConfig::read_command(const State prev_state, DataStorage* ds)
{
	uint32_t byte_ctr = 0;
	char rx_byte = '0';
	State curr_state = prev_state;
	char * err;
	uint32_t eol_len = 2;	//"\r\n"
	PipetteToolConfigErr local_err = PipetteToolConfigErr::NONE;

	// Read till eol char
	while (rx_byte != LF) {
		delay(10);

		if (byte_ctr == BUF_LEN) {
			local_err = PipetteToolConfigErr::BUF_MAX_SIZE;
			byte_ctr = BUF_LEN - 1;
		}
		if(port->available() > 0) {

			rx_byte = port->read();
			buf[byte_ctr] = rx_byte;
			port->print(rx_byte);
			byte_ctr++;
		}
	}

	if (err_get(&local_err)) {
		port->print(CLI_ERROR_STR);
		print_whitespace(1);
		port->print("Input error max buffer reached (");
		port->print(BUF_LEN);
		port->print(")\n");

		return prev_state;
	}

	// Help command
	if (buf[0] == 'e' && byte_ctr == eol_len+1) {
		return State::EXIT;
	}
	else if (buf[0] == 'h' && byte_ctr == eol_len+1) {
		return State::HELP;
	}
	// Print all command
	else if (buf[0] == 'a' && byte_ctr == eol_len+1) {
		return State::PRINT_ALL;
	}
	// Main menu command
	else if (buf[0] == 'm' && byte_ctr == eol_len+1) {
		return State::MAIN_MENU;
	}
	// Read variable command
	else if (buf[0] == 'r' && byte_ctr > eol_len+1) {
		int base = 10;

		uint32_t number = strtoul(&buf[1], &err, base);

		if (buf[1] == *err && number == 0) {
			print_invalid_command();
			return prev_state;
		}
		else if (number >= DataStorage::SIZE) {
			print_invalid_variable_number();
			return prev_state;
		}

		*ds = (DataStorage)number;

		storage_type st;
		DataStorageType dst = data_storage[*ds].storage_type;

		port->println(CLI_VARIABLE_TABLE_LEGEND_STR);
		port->println("----------------------------------------------------------------");
		print_storage_variable(*ds, dst, &st);

		return State::READ;
	}
	// Write variable command
	else if (buf[0] == 'w' && byte_ctr > eol_len+1) {
		int base = 10;

		uint32_t number = strtoul(&buf[1], &err, base);

		if (buf[1] == *err && number == 0) {
			print_invalid_command();
			return prev_state;
		}
		else if (number >= DataStorage::SIZE) {
			print_invalid_variable_number();
			return prev_state;
		}

		*ds = (DataStorage)number;

		return State::WRITE;
	}

	else if (prev_state == State::READ) {


	}
	else if (prev_state == State::WRITE) {
		local_err = PipetteToolConfigErr::NONE;


		if (data_storage[*ds].action_req) {
			storage_type st;
			EncoderInterfaceErr err_encoder = EncoderInterfaceErr::NONE;
			PipetteToolStorageErr err_storage = PipetteToolStorageErr::NONE;

			int32_t pos = pt->encoder->read(&err_encoder);

			if (pt->encoder->err_get(&err_encoder)) {
				port->print(CLI_ERROR_STR);
				print_whitespace(1);
				port->print("Encoder error code: ");
				port->print((uint32_t)err_encoder);
				print_newline(1);
				return prev_state;
			}
			st.i32 = pos;


			switch (*ds) {
				case DataStorage::EP_INIT:
					if (buf[0] == 'n') {
						return State::MAIN_MENU;
					}
					else if(buf[0] != 'y') {
						print_invalid_command();
						return State::MAIN_MENU;
					}

					st.i32 = INIT_ENCODER_POS;
					// No break
				case DataStorage::EP_TOP:
				case DataStorage::EP_TIP:
				case DataStorage::EP_BOT:
					store_data(*ds, st.i32, &err_storage);
					if (PipetteToolStorage::err_get(&err_storage)) {
						port->print(CLI_ERROR_STR);
						print_whitespace(1);
						port->print("Storage error code: ");
						port->print((uint32_t)err_storage);
						print_newline(1);
						return prev_state;
					}
					break;
			}
		}
		else {
			store_write_command(*ds, &local_err);
			if(err_get(&local_err)) {
				return prev_state;
			}
		}
		port->print("Successfully stored variable\n");
		return State::MAIN_MENU;
	}

	else {
		port->print(CLI_ERROR_STR);
		print_whitespace(1);
		port->print("Invalid command");
		print_newline(1);

		return prev_state;
	}

	return prev_state;
}




void PipetteToolConfig::print_invalid_command()
{
	port->print(CLI_ERROR_STR);
	print_whitespace(1);
	port->print("Invalid command");
	print_newline(1);
}

void PipetteToolConfig::print_invalid_variable_number()
{
	port->print(CLI_ERROR_STR);
	print_whitespace(1);
	port->print("Invalid variable number, MUST be between 0 and ");
	port->print(DataStorage::SIZE - 1);
	print_newline(1);
}

void PipetteToolConfig::print_invalid_variable_limit()
{
	port->print(CLI_ERROR_STR);
	print_whitespace(1);
	port->print("Number exceeds maximum limit");
	print_newline(1);
}



void PipetteToolConfig::print_invalid_conversion()
{
	port->print(CLI_ERROR_STR);
	print_whitespace(1);
	port->print("Conversion error");
	print_newline(1);
}




void PipetteToolConfig::start_cli()
{
	State state = State::MAIN_MENU;
	DataStorage ds;
	PipetteToolErr err_pt = PipetteToolErr::NONE;
	EncoderInterfaceErr err_encoder = EncoderInterfaceErr::NONE;

	print_title_menu();

	// Homing initialize for pipette tool (Needed to write to certain storage variables)
	pt->enable_trajectory_systick();
	pt->enable_trajectory_usbserial();
	Serial.print("INITIALIZING PIPETTE TOOL: ");

	pt->do_homing(&err_pt);
	if (pt->err_get(&err_pt)) {
		Serial.print("FAIL ");
		Serial.print("(Pipette tool error code = ");
		Serial.print((uint32_t)err_pt);
		Serial.println(")");
		return;
	}

	// Reset encoder
	pt->encoder->reset(&err_encoder);
	if (pt->encoder->err_get(&err_encoder)) {
		Serial.print("FAIL ");
		Serial.print("(Encoder error code = ");
		Serial.print((uint32_t)err_encoder);
		Serial.println(")");
		return;
	}

	INIT_ENCODER_POS = pt->encoder->read(&err_encoder);
	if (pt->encoder->err_get(&err_encoder)) {
		Serial.print("FAIL ");
		Serial.print("(Encoder error code = ");
		Serial.print((uint32_t)err_encoder);
		Serial.println(")");
		return;
	}

	Serial.println("PASS");

	while (1) {

		switch(state) {
			case State::MAIN_MENU:
				print_command_menu();
				break;
			case State::EXIT:
				print_command_exit();
				return;
			case State::HELP:
				print_command_help();
				state = State::MAIN_MENU;
				break;
			case State::READ:
				print_command_menu();
				state = State::MAIN_MENU;
				break;
			case State::WRITE:
				print_command_write(ds, state);
				break;
			case State::PRINT_ALL:
				print_all_storage_variables();
				print_command_menu();
				state = State::MAIN_MENU;
				break;
		}

		state = read_command(state, &ds);
	}
}

void PipetteToolConfig::print_command_menu()
{
	print_newline(1);
	port->print(CLI_ENTER_COMMAND_STR);
}

void PipetteToolConfig::print_command_help()
{

	print_title_menu();
	/*
	port->print(CLI_ERROR_STR);
	print_whitespace(1);
	port->print("Command not implemented\n");
	*/
	print_command_menu();
}

void PipetteToolConfig::print_command_exit()
{

	port->print("Exiting command line interface ...");
	print_newline(3);
}

void PipetteToolConfig::print_command_write(DataStorage ds, State state)
{
	DriverInterfaceErr err_driver = DriverInterfaceErr::NONE;


	if (data_storage[ds].action_req) {


		switch (ds) {
			case DataStorage::EP_TOP:
				pt->driver->disable_motor(&err_driver);
				port->print("Turn the motor manually to the TOP position (Press ENTER to save): ");
				break;


			case DataStorage::EP_INIT:
				port->print("Store acquired Init encoder position[");
				port->print(INIT_ENCODER_POS);
				port->print("]? (y, n): ");
				break;

			case DataStorage::EP_TIP:
				pt->driver->disable_motor(&err_driver);
				port->print("Turn the motor manually to the start of a pipette TIP position (Press ENTER to save): ");
				break;

			case DataStorage::EP_BOT:
				pt->driver->disable_motor(&err_driver);
				port->print("Turn the motor manually to the BOTTOM position (Press ENTER to save): ");
				break;
		}



	}
	else {
		String storage_type_str = get_data_type_limits_str(data_storage[ds].storage_type);
		print_newline(1);
		port->print("Enter ");
		port->print(data_storage[ds].name);
		port->print(" (");
		port->print(storage_type_str);
		port->print(")");
		port->print(": ");
	}
}



// https://linux.die.net/man/3/strtoull
void PipetteToolConfig::store_write_command(DataStorage idx, PipetteToolConfigErr* p_err) {

	storage_type st;
	char * err;
	DataStorageType dst = data_storage[idx].storage_type;
	int base = 10;

	uint64_t temp_uint64;
	int64_t temp_int64;

	PipetteToolStorageErr storage_err = PipetteToolStorageErr::NONE;



	switch (dst) {
		case DataStorageType::BOOL:
			errno = 0;
			temp_uint64 = strtoull(buf, &err, base);

			if ((*buf == *err && st.b == 0) ||
				 *err != CR)
			{
				*p_err = PipetteToolConfigErr::CONVERSION_ERROR;
				print_invalid_conversion();
				break;
			}
			else if (errno != 0 ||
					temp_uint64 > 1)
			{
				errno = 0;
				*p_err = PipetteToolConfigErr::OUTSIDE_LIMITS;
				print_invalid_variable_limit();
				break;
			}
			st.b = temp_uint64;
			store_data(idx, st.b, &storage_err);
			break;

		case DataStorageType::FLOAT:
			errno = 0;
			st.f = strtof(buf, &err);

			if ((*buf == *err && st.f == 0) ||
				  *err != CR)
			{
				*p_err = PipetteToolConfigErr::CONVERSION_ERROR;
				print_invalid_conversion();
				break;
			}
			else if (errno != 0)
			{
				errno = 0;
				*p_err = PipetteToolConfigErr::OUTSIDE_LIMITS;
				print_invalid_variable_limit();
				break;
			}
			store_data(idx, st.f, &storage_err);
			break;

		case DataStorageType::DOUBLE:
			errno = 0;
			st.d = strtod(buf, &err);

			if ((*buf == *err && st.d == 0) ||
				 *err != CR)
			{
				*p_err = PipetteToolConfigErr::CONVERSION_ERROR;
				print_invalid_conversion();
				break;
			}
			else if (errno != 0)
			{
				errno = 0;
				*p_err = PipetteToolConfigErr::OUTSIDE_LIMITS;
				print_invalid_variable_limit();
				break;
			}
			store_data(idx, st.d, &storage_err);
			break;

		case DataStorageType::UINT8_T:
			errno = 0;
			temp_uint64 = strtoull(buf, &err, base);
			if ((*buf == *err && temp_uint64 == 0) ||
				 *err != CR)
			{
				*p_err = PipetteToolConfigErr::CONVERSION_ERROR;
				print_invalid_conversion();
				break;
			}
			else if (errno != 0 ||
					temp_uint64 > 255)
			{
				errno = 0;
				*p_err = PipetteToolConfigErr::OUTSIDE_LIMITS;
				print_invalid_variable_limit();
				break;
			}
			st.u8 = temp_uint64;
			store_data(idx, st.u8, &storage_err);
			break;

		case DataStorageType::UINT16_T:
			errno = 0;
			temp_uint64 = strtoull(buf, &err, base);
			if ((*buf == *err && temp_uint64 == 0) ||
				 *err != CR)
			{
				*p_err = PipetteToolConfigErr::CONVERSION_ERROR;
				print_invalid_conversion();
				break;
			}
			else if (errno != 0 ||
					temp_uint64 > 65535)
			{
				errno = 0;
				*p_err = PipetteToolConfigErr::OUTSIDE_LIMITS;
				print_invalid_variable_limit();
				break;
			}
			st.u16 = temp_uint64;
			store_data(idx, st.u16, &storage_err);
			break;

		case DataStorageType::UINT32_T:
			errno = 0;
			temp_uint64 = strtoull(buf, &err, base);
			if ((*buf == *err && temp_uint64 == 0) ||
				 *err != CR)
			{
				*p_err = PipetteToolConfigErr::CONVERSION_ERROR;
				print_invalid_conversion();
				break;
			}
			else if (errno != 0 ||
					temp_uint64 > 4294967295)
			{
				errno = 0;
				*p_err = PipetteToolConfigErr::OUTSIDE_LIMITS;
				print_invalid_variable_limit();
				break;
			}
			st.u32 = temp_uint64;
			store_data(idx, st.u32, &storage_err);
			break;

		case DataStorageType::UINT64_T:
			errno = 0;
			temp_uint64 = strtoull(buf, &err, base);
			if ((*buf == *err && temp_uint64 == 0) ||
				 *err != CR)
			{
				*p_err = PipetteToolConfigErr::CONVERSION_ERROR;
				print_invalid_conversion();
				break;
			}
			else if (errno != 0 ||
					temp_uint64 > 18446744073709551615ULL)
			{
				errno = 0;
				*p_err = PipetteToolConfigErr::OUTSIDE_LIMITS;
				print_invalid_variable_limit();
				break;
			}
			st.u64 = temp_uint64;
			store_data(idx, st.u64, &storage_err);
			break;

		case DataStorageType::INT8_T:
			errno = 0;
			temp_int64 = strtoll(buf, &err, base);
			if ((*buf == *err && temp_int64 == 0) ||
				 *err != CR)
			{
				*p_err = PipetteToolConfigErr::CONVERSION_ERROR;
				print_invalid_conversion();
				break;
			}
			else if (errno != 0 ||
					temp_int64 > 127 ||
					temp_int64 < -128)
			{
				errno = 0;
				*p_err = PipetteToolConfigErr::OUTSIDE_LIMITS;
				print_invalid_variable_limit();
				break;
			}
			st.i8 = temp_int64;
			store_data(idx, st.i8, &storage_err);
			break;

		case DataStorageType::INT16_T:
			errno = 0;
			temp_int64 = strtoll(buf, &err, base);
			if ((*buf == *err && temp_int64 == 0) ||
				 *err != CR)
			{
				*p_err = PipetteToolConfigErr::CONVERSION_ERROR;
				print_invalid_conversion();
				break;
			}
			else if (errno != 0 ||
					temp_int64 > 32767 ||
					temp_int64 < -32768)
			{
				errno = 0;
				*p_err = PipetteToolConfigErr::OUTSIDE_LIMITS;
				print_invalid_variable_limit();
				break;
			}
			st.i16 = temp_int64;
			store_data(idx, st.i16, &storage_err);
			break;

		case DataStorageType::INT32_T:
			errno = 0;
			temp_int64 = strtoll(buf, &err, base);
			if ((*buf == *err && temp_int64 == 0) ||
				 *err != CR)
			{
				*p_err = PipetteToolConfigErr::CONVERSION_ERROR;
				print_invalid_conversion();
				break;
			}
			else if (errno != 0 ||
					temp_int64 > 2147483647 ||
					temp_int64 < -2147483648)
			{
				errno = 0;
				*p_err = PipetteToolConfigErr::OUTSIDE_LIMITS;
				print_invalid_variable_limit();
				break;
			}
			st.i32 = temp_int64;
			store_data(idx, st.i32, &storage_err);
			break;

		case DataStorageType::INT64_T:
			errno = 0;
			temp_int64 = strtoll(buf, &err, base);
			if ((*buf == *err && temp_int64 == 0) ||
				 *err != CR)
			{
				*p_err = PipetteToolConfigErr::CONVERSION_ERROR;
				print_invalid_conversion();
				break;
			}
			else if (errno != 0 ||
					temp_int64 > 9223372036854775807 ||
					temp_int64 < -9223372036854775808)
			{
				errno = 0;
				*p_err = PipetteToolConfigErr::OUTSIDE_LIMITS;
				print_invalid_variable_limit();
				break;
			}
			st.i64 = temp_int64;
			store_data(idx, st.i64, &storage_err);
			break;
	}

	if(err_get(p_err)) {return;}

	if (PipetteToolStorage::err_get(&storage_err)) {
		*p_err = PipetteToolConfigErr::STORAGE_ERROR;
		port->print(CLI_ERROR_STR);
		print_whitespace(1);
		port->print("Storage err code: ");
		port->print((uint32_t)storage_err);
		print_newline(1);



		return;
	}

}


















void PipetteToolConfig::print_title_menu(){
	print_newline(1);
	port->print("################################################################\n");
	port->print("#\n");
	port->print("#  "); port->println(CLI_TITLE_STR);
	port->print("#\n");
	port->print("#\n");
	port->print("#  "); port->println(CLI_AVAILABLE_COMMANDS_STR);
	port->print("#  "); port->println(CLI_COMMAND_HELP_STR);
	port->print("#  "); port->println(CLI_COMMAND_READ_STR);
	port->print("#  "); port->println(CLI_COMMAND_WRITE_STR);
	port->print("#  "); port->println(CLI_COMMAND_PRINT_ALL_STR);
	port->print("#  "); port->println(CLI_COMMAND_MENU_STR);
	port->print("#  "); port->println(CLI_COMMAND_EXIT_STR);
	port->print("################################################################\n");
	print_newline(1);
}



String PipetteToolConfig::get_data_type_limits_str(DataStorageType dst) {

	switch (dst) {
		case DataStorageType::BOOL:
			return "0 - 1 (false, true)";

		case DataStorageType::FLOAT:
			return "float number";

		case DataStorageType::DOUBLE:
			return "double number";

		case DataStorageType::UINT8_T:
			return "0 - 255";

		case DataStorageType::UINT16_T:
			return "0 - 65,535";

		case DataStorageType::UINT32_T:
			return "0 - 4,294,967,295";

		case DataStorageType::UINT64_T:
			return "0 - 18,446,744,073,709,551,615";

		case DataStorageType::INT8_T:
			return "-128 - 127";

		case DataStorageType::INT16_T:
			return "-32,768 - 32,767";

		case DataStorageType::INT32_T:
			return "-2,147,483,648 - 2,147,483,647";

		case DataStorageType::INT64_T:
			return "−9,223,372,036,854,775,808 - 9,223,372,036,854,775,807";
	}
}


String PipetteToolConfig::get_data_type_str(DataStorageType dst) {

	switch (dst) {
		case DataStorageType::BOOL:
			return "bool";

		case DataStorageType::FLOAT:
			return "float";

		case DataStorageType::DOUBLE:
			return "double";

		case DataStorageType::UINT8_T:
			return "uint8_t";

		case DataStorageType::UINT16_T:
			return "uint16_t";

		case DataStorageType::UINT32_T:
			return "uint32_t";

		case DataStorageType::UINT64_T:
			return "uint64_t";

		case DataStorageType::INT8_T:
			return "int8_t";

		case DataStorageType::INT16_T:
			return "int16_t";

		case DataStorageType::INT32_T:
			return "int32_t";

		case DataStorageType::INT64_T:
			return "int64_t";
	}
}




void PipetteToolConfig::print_whitespace(uint32_t nb)
{
	for (uint32_t i = 0; i < nb;i++) {
		port->print(' ');
	}
}

void PipetteToolConfig::print_newline(uint32_t nb)
{
	for (uint32_t i = 0; i < nb;i++) {
		port->print('\n');
	}
}

void PipetteToolConfig::print_tab(uint32_t nb)
{
	for (uint32_t i = 0; i < nb;i++) {
		port->print('\t');
	}
}


void PipetteToolConfig::print_all_storage_variables()
{
	storage_type st;

	port->println(CLI_VARIABLE_TABLE_LEGEND_STR);
	port->println("----------------------------------------------------------------");

	// Loop through all storage values
	for(uint32_t i = 0; i < DataStorage::SIZE; i++) {


		print_storage_variable(PipetteToolStorage::DataStorage(i), data_storage[i].storage_type, &st);
		//print_newline(1);

	}
}



void PipetteToolConfig::print_available_commands()
{
	port->println(CLI_AVAILABLE_COMMANDS_STR);
	port->println(CLI_COMMAND_HELP_STR);
	port->println(CLI_COMMAND_READ_STR);
	port->println(CLI_COMMAND_WRITE_STR);
	port->println(CLI_COMMAND_PRINT_ALL_STR);
	port->println(CLI_COMMAND_MENU_STR);
}





void PipetteToolConfig::print_storage_variable(DataStorage ds, DataStorageType dst, storage_type* st) {
	port->print(ds);
	print_tab(1);
	port->print(data_storage[ds].name);


	switch(ds) {
		case DataStorage::PT_TYPE:
			print_tab(3);
			break;
		case DataStorage::PT_SN:
			print_tab(2);
			break;
		case DataStorage::PT_LEAD:
			print_tab(3);
			break;
		case DataStorage::EP_TOP:
			print_tab(2);
			break;
		case DataStorage::EP_INIT:
			print_tab(1);
			break;
		case DataStorage::EP_TIP:
			print_tab(2);
			break;
		case DataStorage::EP_BOT:
			print_tab(1);
			break;
		case DataStorage::EP_MAX:
			print_tab(1);
			break;
		case DataStorage::EP_MIN:
			print_tab(1);
			break;
	}



	port->print('(');
	port->print(get_data_type_str(data_storage[ds].storage_type));
	port->print(')');



	switch (dst) {
		case DataStorageType::BOOL:
			print_tab(2);
			break;

		case DataStorageType::FLOAT:
			print_tab(2);
			break;

		case DataStorageType::DOUBLE:
			print_tab(1);
			break;

		case DataStorageType::UINT8_T:
			print_tab(1);
			break;

		case DataStorageType::UINT16_T:
			print_tab(1);
			break;

		case DataStorageType::UINT32_T:
			print_tab(1);
			break;
		case DataStorageType::UINT64_T:
			print_tab(1);
			break;

		case DataStorageType::INT8_T:
			print_tab(1);
			break;

		case DataStorageType::INT16_T:
			print_tab(1);
			break;

		case DataStorageType::INT32_T:
			print_tab(1);
			break;

		case DataStorageType::INT64_T:
			print_tab(1);
			break;
	}



	PipetteToolStorageErr storage_err = PipetteToolStorageErr::NONE;


	switch (dst) {
		case DataStorageType::BOOL:
			retrieve_data(ds, &st->b, &storage_err);
			if (PipetteToolStorage::err_get(&storage_err)) {break;}
			port->print(st->b);
			break;

		case DataStorageType::FLOAT:
			retrieve_data(ds, &st->f, &storage_err);
			if (PipetteToolStorage::err_get(&storage_err)) {break;}
			port->print(st->f, 16);
			break;

		case DataStorageType::DOUBLE:
			retrieve_data(ds, &st->d, &storage_err);
			if (PipetteToolStorage::err_get(&storage_err)) {break;}
			port->print(st->d, 16);
			break;

		case DataStorageType::UINT8_T:
			retrieve_data(ds, &st->u8, &storage_err);
			if (PipetteToolStorage::err_get(&storage_err)) {break;}
			port->print(st->u8);
			break;

		case DataStorageType::UINT16_T:
			retrieve_data(ds, &st->u16, &storage_err);
			if (PipetteToolStorage::err_get(&storage_err)) {break;}
			port->print(st->u16);
			break;

		case DataStorageType::UINT32_T:
			retrieve_data(ds, &st->u32, &storage_err);
			if (PipetteToolStorage::err_get(&storage_err)) {break;}
			port->print(st->u32);
			break;

		case DataStorageType::UINT64_T:
			retrieve_data(ds, &st->u64, &storage_err);
			if (PipetteToolStorage::err_get(&storage_err)) {break;}
			print_uint64_t(st->u64);	// Arduino dosen't have overloaded print function for uint64_t
			break;

		case DataStorageType::INT8_T:
			retrieve_data(ds, &st->i8, &storage_err);
			if (PipetteToolStorage::err_get(&storage_err)) {break;}
			port->print(st->i8);
			break;

		case DataStorageType::INT16_T:
			retrieve_data(ds, &st->i16, &storage_err);
			if (PipetteToolStorage::err_get(&storage_err)) {break;}
			port->print(st->i16);
			break;

		case DataStorageType::INT32_T:
			retrieve_data(ds, &st->i32, &storage_err);
			if (PipetteToolStorage::err_get(&storage_err)) {break;}
			port->print(st->i32);
			break;

		case DataStorageType::INT64_T:
			retrieve_data(ds, &st->i64, &storage_err);
			if (PipetteToolStorage::err_get(&storage_err)) {break;}
			print_int64_t(st->i64);	// Arduino dosen't have overloaded print function for int64_t

			break;
	}

	if (PipetteToolStorage::err_get(&storage_err)) {
		if(storage_err == PipetteToolStorageErr::OPEN_FILE_FAILED) {
			port->print('*');
			port->print(CLI_NOT_STORED_STR);
		}
		else {
			port->print("*Error code = ");
			port->print((uint8_t)storage_err);
		}
	}

	print_newline(1);


	// Reset error
	//PipetteToolStorage::ec = PipetteToolStorage::ErrorCode::NONE;
}




bool PipetteToolConfig::err_get(PipetteToolConfigErr* p_err)
{
	if (*p_err == PipetteToolConfigErr::NONE) {
		return false;
	}
	return true;
}

void PipetteToolConfig::err_clear(PipetteToolConfigErr* p_err)
{
	*p_err = PipetteToolConfigErr::NONE;
}










// smart memory use -> increase performance
// http://forum.arduino.cc/index.php/topic,143584.0.html
size_t PipetteToolConfig::print_uint64_t(uint64_t number64, int base)	// base to uint8_t
{
	size_t n = 0;
	unsigned char buf[64];
	uint8_t i = 0;
	uint8_t ml=3;

	union{
		uint64_t n64;
		struct
		{
			uint32_t a[2];	// 0 = LOW byte  1 = HIGH byte of the uint64_t
		}
		n32;
		struct
		{
			uint16_t a[4];
		}
		n16;
	}
	nn;

	if (number64 == 0)
	{
		port->write('0'); // write is faster than print.
		return 1;
	}

	uint16_t th16 = 1 * base * base * base;
	if (base < 16) {
		th16 *= base;
		ml++;
	}
	if (base < 8) {
		th16 *= base;
		ml++;
	}

	nn.n64 = number64;

	while (nn.n32.a[1] > 0)	// while 64.HIGH > 0; as long as we need 64 bit math; !! but a 32bit test
	{
		uint64_t q = nn.n64 / th16;	// split of max # bases
		uint16_t r = nn.n64 - q*th16;
		for (uint8_t j=0; j<ml; j++)	// process these 16 bit wise
		{
			uint16_t qq = r/base;
			buf[i++] = r - qq*base;
			r = qq;
		}
		nn.n64 = q;
	}

	//uint32_t number32 = nn.n64;
	while (nn.n16.a[1] > 0) // as long as we need 32 bit math
	{
		uint32_t q = nn.n32.a[0] / th16;// split of max # bases
		uint16_t r = nn.n32.a[0] - q*th16;
		for (uint8_t j=0; j<ml; j++) // process these 16 bit wise
		{
			uint16_t qq = r/base;
			buf[i++] = r - qq*base;
			r = qq;
		}
		nn.n32.a[0] = q;
	}

	while (nn.n16.a[0] > 0) // process the remaining
	{
		uint16_t qq = nn.n16.a[0]/base;
		buf[i++] = nn.n16.a[0] - qq*base;
		nn.n16.a[0] = qq;
	}

	n = i;
	if (base >10)
	{
		for (; i > 0; i--)
			port->write((char) (buf[i - 1] < 10 ?
					'0' + buf[i - 1] :
					'A' + buf[i - 1] - 10));
	}
	else
	{
		for (; i > 0; i--) port->write((char) ('0' + buf[i - 1]));
	}
 return n;
}


size_t PipetteToolConfig::print_int64_t(int64_t number, int base)
{
 size_t n = 0;
 if (number < 0)
 {
	port->write('-');
	number = -number;
	n++;
 }
 n += print_uint64_t((uint64_t)number, base);
 return n;
}




/*
size_t Print::print(uint64_t number, int base)
{
 size_t n = 0;
 unsigned char buf[64];
 uint8_t i = 0;

 if (number == 0)
 {
   n += print((char)'0');
   return n;
 }

 if (base < 2) base = 2;
 else if (base > 16) base = 16;

 while (number > 0)
 {
   uint64_t q = number/base;
   buf[i++] = number - q*base;
   number = q;
 }

 for (; i > 0; i--)
   n += write((char) (buf[i - 1] < 10 ?
     '0' + buf[i - 1] :
     'A' + buf[i - 1] - 10));

  return n;
}

*/



#endif // ARDUINO






