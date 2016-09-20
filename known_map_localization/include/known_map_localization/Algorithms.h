/*
 * Algorithms.h
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_ALGORITHMS_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_ALGORITHMS_H_

namespace known_map_localization {

/**
 * Defines algorithm identifiers for different aligning algorithm choices.
 */
typedef enum {
	MAPSTITCH = 0,//!< MAPSTITCH Uses the mapstitch package
	MAPMERGE = 1, //!< MAPMERGE Uses the mapmerge library by Stefano Carpin
	CSMERGE = 2   //!< CSMERGE Uses the cs_merge package (not yet implemented)
} Algorithm;

}

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_ALGORITHMS_H_ */
