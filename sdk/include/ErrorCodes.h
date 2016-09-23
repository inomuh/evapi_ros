#ifdef English_dox
/**
 * \file   ErrorCodes.h
 * \author Fatih İnan (fatih.inan@inovasyonmuhendislik.com)
 * \date   Mar, 2016
 * \brief  Returns error description which occurs in sdk or any other packages.
 * \details
 * Returns a string in this format -> [Error Code <number>] <package>: <description>.
 * <number> is error code number.
 * <package> is package which error occurs in it.
 * <description> is describes why this error is occurred.
 * It is used to get description of an error by using id.
 */
#endif

#include <string>

#ifdef English_dox
//! Returns error description.
/**
 * \param i_error_code is id of the error which thrown from try block.
 * \return Description of the error in this format -> [Error Code <number>] <package>: <description>.
 */
#endif
std::string GetErrorDescription(int i_error_code);
