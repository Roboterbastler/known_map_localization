/*
 * Exception.h
 *
 *  Created on: 05.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_EXCEPTION_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_EXCEPTION_H_

#include <stdexcept>

namespace known_map_localization {

class KnownMapLocalizationException : public std::runtime_error {
public:
	KnownMapLocalizationException(std::string msg) : std::runtime_error(msg) {};
};

class AlignmentNotAvailable : public KnownMapLocalizationException {
public:
	AlignmentNotAvailable(std::string msg) : KnownMapLocalizationException(msg) {};
};

class IllegalAlgorithm : public KnownMapLocalizationException {
public:
	IllegalAlgorithm(std::string msg) : KnownMapLocalizationException(msg) {};
};

class AlgorithmNotSpecified : public KnownMapLocalizationException {
public:
	AlgorithmNotSpecified(std::string msg) : KnownMapLocalizationException(msg) {};
};

} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_EXCEPTION_H_ */
