/*
 * Filter.h
 *
 *  Created on: 02.08.2016
 *      Author: jacob
 */

#ifndef KNOWN_MAP_LOCALIZATION_INCLUDE_FILTER_FILTER_H_
#define KNOWN_MAP_LOCALIZATION_INCLUDE_FILTER_FILTER_H_

#include <vector>
#include <exception>

#include "alignment/StampedAlignment.h"

namespace known_map_localization {
namespace filter {

class Filter {
public:
	Filter();
	virtual ~Filter();

	virtual void addAlignment(alignment::StampedAlignment alignment) = 0;

	/**
	 *
	 * @return
	 * @throw AlignmentNotAvailable
	 */
	const alignment::Alignment& getAlignment() const;

protected:
	std::vector<alignment::StampedAlignment> alignments;
	alignment::Alignment filteredAlignment;
	bool ready;
};

class AlignmentNotAvailable : public std::runtime_error {};

typedef boost::shared_ptr<Filter> FilterPtr;
typedef boost::shared_ptr<Filter const> FilterConstPtr;
} /* namespace filter */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_FILTER_FILTER_H_ */
