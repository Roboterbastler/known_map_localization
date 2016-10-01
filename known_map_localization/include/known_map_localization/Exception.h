/*
 * Exception.h
 *
 *  Created on: 05.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_EXCEPTION_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_EXCEPTION_H_

#include <stdexcept>

namespace kml {

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

class AlignerException : public KnownMapLocalizationException {
public:
	AlignerException(std::string msg) : KnownMapLocalizationException(msg) {};
};

class AlignerMissingParameter : public AlignerException {
public:
	AlignerMissingParameter(std::string msg) : AlignerException(msg) {};
};

class AlignerInternalError : public AlignerException {
public:
	AlignerInternalError(std::string msg) : AlignerException(msg) {};
};

class AlignerFailed : public AlignerException {
public:
	AlignerFailed(std::string msg) : AlignerException(msg) {};
};

class ScaleNotAvailable : public KnownMapLocalizationException {
public:
	ScaleNotAvailable(std::string msg) : KnownMapLocalizationException(msg) {};
};

class DifferentUTMGridZones : public KnownMapLocalizationException {
public:
	DifferentUTMGridZones(std::string msg) : KnownMapLocalizationException(msg) {};
};

class InvalidAssociation : public KnownMapLocalizationException {
public:
	InvalidAssociation(std::string msg) : KnownMapLocalizationException(msg) {};
};

} /* namespace kml */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_EXCEPTION_H_ */
