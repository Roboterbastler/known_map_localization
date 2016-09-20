/*
 * AlgorithmSelector.h
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_ALGORITHMSELECTOR_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_ALGORITHMSELECTOR_H_

#include <boost/shared_ptr.hpp>

#include <Algorithms.h>
#include <aligning/Aligner.h>
#include <preprocessing/KnownMapPreprocessor.h>
#include <preprocessing/SlamMapPreprocessor.h>

namespace known_map_localization {

class AlgorithmSelector;
typedef boost::shared_ptr<AlgorithmSelector> AlgorithmSelectorPtr;
typedef boost::shared_ptr<AlgorithmSelector const> AlgorithmSelectorConstPtr;

/**
 * # Algorithm Selector
 * Provides methods to read the algorithm type set by parameter.
 *
 * ## Parameters
 * - **algorithm**: The selected algorithm
 */
class AlgorithmSelector {
public:
	/**
	 * Tries to read the **algorithm** parameter and returns the corresponding identifier.
	 * @return The algorithm identifier
	 * @throwsAlgorithmNotSpecified If the parameter is not given
	 * @throws IllegalAlgorithm If the parameter is not a legal algorithm specifier
	 */
	static Algorithm determineAlgorithm();

	/**
	 * Evaluates the algorithm name string and returns the corresponding algorithm identifier.
	 * @param algorithm The name of the algorithm
	 * @return The corresponding algorithm identifier
	 * @throws IllegalAlgorithm If the parameter is not a legal algorithm specifier
	 */
	static Algorithm getAlgorithm(std::string algorithm);
};

} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_ALGORITHMSELECTOR_H_ */
