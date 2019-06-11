/*
 * pipette_tool_config.h
 *
 *  Created on: Nov 19, 2018
 *      Author: biobot
 */

#ifndef LIB_BIOBOT_BIOBOT_PIPETTE_TOOL_ARDUINO_PIPETTE_TOOL_CONFIG_H_
#define LIB_BIOBOT_BIOBOT_PIPETTE_TOOL_ARDUINO_PIPETTE_TOOL_CONFIG_H_


#if defined(ARDUINO)

#include <robotic_tool/pipette_tool/arduino/pipette_tool_storage.h>

#include <robotic_tool/pipette_tool/pipette_tool.h>



enum class PipetteToolConfigErr : uint8_t {
	NONE,
	BUF_MAX_SIZE,
	CONVERSION_ERROR,
	OUTSIDE_LIMITS,
	STORAGE_ERROR,
};












class PipetteToolConfig : public PipetteToolStorage {
public:



	PipetteToolConfig(Serial_* port, uint32_t baudrate, PipetteTool* pt);

	Serial_* port;
	uint32_t baudrate;
	PipetteTool* pt;


	enum class State : uint8_t {
		MAIN_MENU,
		HELP,
		READ,
		WRITE,
		PRINT_ALL,
		EXIT
	};

	void start_cli();
	bool err_get(PipetteToolConfigErr* p_err);
	void err_clear(PipetteToolConfigErr* p_err);


private:

	const char LF = '\n';
	const char CR = '\r';

	State read_command(const State state, DataStorage* ds);
	//State state = State::MAIN_MENU;

	String get_data_type_str(DataStorageType);
	String get_data_type_limits_str(DataStorageType);




	void print_whitespace(uint32_t nb);
	void print_newline(uint32_t nb);
	void print_tab(uint32_t nb);
	void print_all_storage_variables();
	void print_available_commands();

	void print_title_menu();

	void print_command_menu();
	void print_command_help();
	void print_command_exit();
	void print_command_write(DataStorage ds, State state);

	void print_invalid_command();
	void print_invalid_variable_number();
	void print_invalid_variable_limit();
	void print_invalid_conversion();

	void store_write_command(DataStorage ds, PipetteToolConfigErr* p_err);

	void print_storage_variable(DataStorage ds, DataStorageType dst, storage_type* st);
	size_t print_uint64_t(uint64_t number64, int base=DEC);
	size_t print_int64_t(int64_t number, int base=DEC);

	static const uint32_t  BUF_LEN = 32;
	char buf[BUF_LEN];


	int32_t INIT_ENCODER_POS = 0;


	const char* CLI_TITLE_STR = 					"Pipette Tool Storage CLI (v1.0.0)";
	const char* CLI_VARIABLE_TABLE_LEGEND_STR = 	"N\tNAME\t\t\tTYPE\t\tVALUE";
	const char* CLI_AVAILABLE_COMMANDS_STR = 		"Available commands:";
	const char* CLI_COMMAND_HELP_STR = 				"\th\t\tHelp.";
	const char* CLI_COMMAND_MENU_STR = 				"\tm\t\tReturn to main menu.";
	const char* CLI_COMMAND_PRINT_ALL_STR = 		"\ta\t\tPrint all variables.";
	const char* CLI_COMMAND_READ_STR = 				"\tr0 - rN\t\tRead stored variable N.";
	const char* CLI_COMMAND_WRITE_STR = 			"\tw0 - wN\t\tWrite to stored variable N.";
	const char* CLI_COMMAND_EXIT_STR = 				"\te\t\tExit.";



	const char* CLI_NOT_STORED_STR = 				"NOT_STORED";
	const char* CLI_ENTER_STR = 					"Enter";
	const char* CLI_ENTER_COMMAND_STR = 					"Enter command: ";

	const char* CLI_SUCCESS_STORE_STR = 			"Succesfully stored";
	const char* CLI_ERROR_STR = 					"--- ERROR --- ";
};

#endif	// ARDUINO

#endif /* LIB_BIOBOT_BIOBOT_PIPETTE_TOOL_ARDUINO_PIPETTE_TOOL_CONFIG_H_ */
