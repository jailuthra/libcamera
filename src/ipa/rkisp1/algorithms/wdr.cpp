/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025, Ideas On Board
 *
 * RkISP1 Wide Dynamic Range control
 */

#include "wdr.h"

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include "libcamera/internal/yaml_parser.h"

#include "linux/rkisp1-config.h"

/**
 * \file wdr.h
 */

namespace libcamera {

namespace ipa::rkisp1::algorithms {

/**
 * \class WideDynamicRange
 * \brief RkISP1 Wide Dynamic Range control
 *
 * The curves are specified in the tuning data and defined using 33 points.
 *
 * - The X coordinates are expressed using 32 intervals, with the first point
 *   at X coordinate 0. Each interval is expressed as a 3-bit value DY (from
 *   WDR_DY_1 to WDR_DY_32), stored in the RKISP1_CIF_ISP_WDR_TONECURVE_1 to
 *   RKISP1_CIF_ISP_WDR_TONECURVE_4  registers. The real interval is equal to
 *   \f$2^{dy+3}\f$.
 *
 * - The Y coordinates are specified as 33 values, with a 12-bit resolution.
 *   Each value must be in the [-2048, 2047] range compared to the previous
 *   value.
 */

LOG_DEFINE_CATEGORY(RkISP1Wdr)

static constexpr unsigned int kTonecurveXIntervals = 32;

WideDynamicRange::WideDynamicRange()
{
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int WideDynamicRange::init([[maybe_unused]] IPAContext &context,
				   const YamlObject &tuningData)
{
	std::vector<uint16_t> xIntervals =
		tuningData["x-intervals"].getList<uint16_t>().value_or(std::vector<uint16_t>{});
	if (xIntervals.size() != kTonecurveXIntervals) {
		LOG(RkISP1Wdr, Error)
			<< "Invalid 'x' coordinates: expected "
			<< kTonecurveXIntervals << " elements, got "
			<< xIntervals.size();

		return -EINVAL;
	}

	/* Compute toneCurveXInterv_ intervals from xIntervals values */
	for (unsigned int i = 0; i < RKISP1_CIF_ISP_WDR_CURVE_NUM_DY_REGS; i++)
		toneCurveXInterv_[i] = 0;
	for (unsigned int i = 0; i < kTonecurveXIntervals; ++i)
		toneCurveXInterv_[i / 8] |= (xIntervals[i] & 0x07) << ((i % 8) * 4);

	toneCurveY_ = tuningData["y"].getList<uint16_t>().value_or(std::vector<uint16_t>{});
	if (toneCurveY_.size() != RKISP1_CIF_ISP_WDR_CURVE_NUM_COEFF) {
		LOG(RkISP1Wdr, Error)
			<< "Invalid 'y' coordinates: expected "
			<< RKISP1_CIF_ISP_WDR_CURVE_NUM_COEFF
			<< " elements, got " << toneCurveY_.size();
		return -EINVAL;
	}

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void WideDynamicRange::prepare([[maybe_unused]] IPAContext &context,
				       const uint32_t frame,
				       [[maybe_unused]] IPAFrameContext &frameContext,
				       RkISP1Params *params)
{
	if (frame > 0)
		return;

	auto config = params->block<BlockType::Wdr>();
	config.setEnabled(true);

	config->dmin_strength = 0x10;

	for (unsigned int i = 0; i < RKISP1_CIF_ISP_WDR_CURVE_NUM_DY_REGS; i++)
		config->tone_curve.dY[i] = toneCurveXInterv_[i];

	std::copy(toneCurveY_.begin(), toneCurveY_.end(), config->tone_curve.ym);
}

REGISTER_IPA_ALGORITHM(WideDynamicRange, "WideDynamicRange")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
