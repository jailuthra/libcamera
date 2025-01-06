/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * RkISP1 Wide Dynamic Range control
 */

#pragma once

#include "linux/rkisp1-config.h"
#include "algorithm.h"

namespace libcamera {

namespace ipa::rkisp1::algorithms {

class WideDynamicRange : public Algorithm
{
public:
	WideDynamicRange();
	~WideDynamicRange() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     RkISP1Params *params) override;

private:
	uint32_t toneCurveXInterv_[RKISP1_CIF_ISP_WDR_CURVE_NUM_DY_REGS];
	std::vector<uint16_t> toneCurveY_;
};

} /* namespace ipa::rkisp1::algorithms */
} /* namespace libcamera */
