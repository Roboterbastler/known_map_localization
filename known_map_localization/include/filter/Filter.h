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

/**
 * # Filter
 */
class Filter {
public:
	Filter();
	virtual ~Filter();

	/**
	 * Add a new alignment and update the filtered alignment accordingly.
	 * Multiple possible implementations are given by subclasses of Filter.
	 * @param alignment The new alignment
	 */
	virtual void addAlignment(alignment::StampedAlignment alignment) = 0;

	/**
	 * Get the filtered alignment, if available. If it isn't available, an exception is thrown.
	 * @return The filtered alignment
	 * @throw AlignmentNotAvailable, if filtered alignment is not available
	 */
	const alignment::Alignment& getAlignment() const;

protected:
	/// list of cached alignments used for filtering
	std::vector<alignment::StampedAlignment> alignments;

	/// the current filtered alignment
	alignment::Alignment filteredAlignment;

	/// flag indicating if filtered alignment is available
	bool ready;
};

typedef boost::shared_ptr<Filter> FilterPtr;
typedef boost::shared_ptr<Filter const> FilterConstPtr;
} /* namespace filter */
} /* namespace known_map_localization */

#endif /* KNOWN_MAP_LOCALIZATION_INCLUDE_FILTER_FILTER_H_ */
