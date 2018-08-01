/*
 * error_handle_register.h
 *
 *  Created on: Jun 5, 2018
 *      Author: robotic_tools
 */

#ifndef ERROR_HANDLE_REGISTER_H_
#define ERROR_HANDLE_REGISTER_H_


class ErrorHandleRegister
{
public:


	ErrorHandleRegister();






	enum Registers {
		REGISTER_ERROR_HANDLE,

		SIZE	/* MUST be last item in enum  */
	};
};

#endif /* ERROR_HANDLE_REGISTER_H_ */
