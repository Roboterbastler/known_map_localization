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

class AlgorithmSelector {
public:
	/**
	 * @throw ros::InvalidNameException
	 */
	AlgorithmSelector();

	preprocessing::SlamMapPreprocessorPtr getSlamMapPreprocessor() const;
	preprocessing::KnownMapPreprocessorPtr getKnownMapPreprocessor() const;
	aligning::AlignerPtr getAligner() const;

private:
	/**
	 * Evaluates the algorithm name string and returns the corresponding enum value.
	 * @param algorithm The name of the algorithm
	 * @return The corresponding enum value
	 */
	static Algorithm getAlgorithm(std::string algorithm);

private:
	preprocessing::SlamMapPreprocessorPtr slamMapPreprocessor;
	preprocessing::KnownMapPreprocessorPtr knownMapPreprocessor;
	aligning::AlignerPtr aligner;
};

typedef boost::shared_ptr<AlgorithmSelector> AlgorithmSelectorPtr;
typedef boost::shared_ptr<AlgorithmSelector const> AlgorithmSelectorConstPtr;
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_ALGORITHMSELECTOR_H_ */
