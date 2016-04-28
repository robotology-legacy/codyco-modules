/* Some string helper functions.
 *
 * Copyright (c) 2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under public domain.
 */

#ifndef _STRING_UTILS_H
#define _STRING_UTILS_H

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

const std::string whitespaces_std (" \t\n\r");
const std::string invalid_name_characters = "{}[],;: \r\n\t#";

inline std::string strip_comments (const std::string &line) {
	return line.substr (0, line.find ('#'));
}

inline std::string strip_whitespaces (const std::string &line, std::string whitespaces = whitespaces_std) {
	std::string result (line);
	if (result.find_first_of (whitespaces) != std::string::npos) {
		result = result.substr (result.find_first_not_of (whitespaces), result.size());
	}

	while (whitespaces.find (result[result.size() - 1]) != std::string::npos) {
		result = result.substr (0, result.size() - 1);
	}
	return result;
}

inline std::string tolower (const std::string &line) {
	std::string result (line);
	for (size_t i = 0; i < line.size(); i++) 
		result[i] = tolower(result[i]);

	return result;
}

inline std::string trim_line (const std::string &line) {
	return tolower (strip_whitespaces (strip_comments (line)));
}

inline std::vector<std::string> tokenize (const std::string &line_in, std::string delimiter=whitespaces_std) {
	std::vector<std::string> result;
	std::string line = line_in;

	while (line.find_first_of (delimiter) != std::string::npos) {
		std::string token = line.substr (0, line.find_first_of (delimiter));

		line = line.substr (token.size() + 1, line.size());
		result.push_back (token);
	}

	if (line.size() > 0)
		result.push_back (line);

	return result;
}

inline std::vector<std::string> tokenize_strip_whitespaces (const std::string &line_in, std::string delimiter=",\t\r\n", std::string whitespaces=whitespaces_std) {
	std::vector<std::string> result;
	std::string line = line_in;

	while (line.find_first_of (delimiter) != std::string::npos) {
		std::string token = line.substr (0, line.find_first_of (delimiter));

		line = line.substr (token.size() + 1, line.size());
		result.push_back (strip_whitespaces(token, whitespaces));
	}

	if (line.size() > 0)
		result.push_back (strip_whitespaces(line, whitespaces));

	return result;
}

inline std::vector<std::string> tokenize_csv_strip_whitespaces (const std::string &line_in, const std::string whitespaces=whitespaces_std) {
	std::vector<std::string> result;
	std::string line = line_in;

	size_t search_start = 0;

	// first replace all whitespaces by regular spaces to simplify things
	search_start = line.find_first_of (whitespaces);
	while (search_start != std::string::npos) {
		line[search_start] = ' ';
		search_start = line.find_first_of (whitespaces, search_start + 1);
	}

	search_start = 0;

	while (line.size() > 0) {
		size_t separator_pos = std::string::npos;
	
		separator_pos = line.find (", ");
		// std::cout << "  search '" << ", " << "' in '" << search_start << "' found at " << separator_pos << std::endl;

		if (separator_pos == std::string::npos) {
			separator_pos = line.size();
		}

		std::string token = line.substr (0, separator_pos);
		// std::cout << "token = '" << token << "' separator_pos = " << separator_pos << std::endl;
		token = strip_whitespaces (token, whitespaces);
		if (token[token.size() - 1] == ',')
			token = token.substr (0, token.size() - 1);
		result.push_back (token);

		if (line.size() <= separator_pos + 2)
			break;

		line = line.substr (separator_pos + 2, line.size());
	}

	return result;
}


/** Counts the number of occurrences of a list of characters.
 *
 * \param line_in The hay-stack to search for.
 * \param characters The needles.
 * \return The sum of occurrences of all needles found in the hay-stack.
 */
inline int count_char (const std::string &line_in, const std::string characters) {
	int count = 0;
	size_t index = 0;
	size_t char_pos = line_in.find_first_of (characters, index);

	while (char_pos != std::string::npos) {
		index = char_pos + 1;
		count ++;
		char_pos = line_in.find_first_of (characters, index) ;
	}

	return count;
}

inline bool is_numeric (const std::string &line) {
	for (unsigned int i = 0; i < line.size(); i++) {
		if (!isdigit(line[i]))
			return false;
	}

	return true;
}

inline std::string sanitize_name (const std::string &name) {
	std::string name_sanitized = name;
	if (is_numeric(name)) {
		std::cerr << "Warning invalid name '" << name << "': name should not be numeric only!" << std::endl;
		name_sanitized = std::string("_") + name;
	}

	// check for invalid characters
	if (name.find_first_of (invalid_name_characters) != std::string::npos) {
		std::cerr << "Error: Found invalid character '"
			<< name[name.find_first_of (invalid_name_characters)]
			<< "' in name '" << name << "'!" << std::endl;
		abort();
	}

	return name_sanitized;
}


#endif
