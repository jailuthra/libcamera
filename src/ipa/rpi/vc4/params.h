/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas On Board
 *
 * Raspberry Pi VC4/BCM2835 ISP Parameters
 */

#pragma once

#include <linux/bcm2835-isp.h>

#include <libipa/v4l2_params.h>

namespace libcamera {

namespace ipa::RPi {

enum class BlockType {
	BlackLevel,
	Geq,
	Gamma,
	Denoise,
	Sharpen,
	Dpc,
	Cdn,
	CcMatrix,
	LensShading,
	AwbGains,
	DGain,
};

namespace details {

template<BlockType B>
struct block_type {
};

#define BCM2835_DEFINE_BLOCK_TYPE(id, structName, blockId)                \
	template<>                                                        \
	struct block_type<BlockType::id> {                                \
		using type = struct bcm2835_isp_params_##structName;      \
		static constexpr bcm2835_isp_param_block_type blockType = \
			BCM2835_ISP_PARAM_BLOCK_##blockId;                \
	};

BCM2835_DEFINE_BLOCK_TYPE(BlackLevel, black_level, BLACK_LEVEL)
BCM2835_DEFINE_BLOCK_TYPE(Geq, geq, GEQ)
BCM2835_DEFINE_BLOCK_TYPE(Gamma, gamma, GAMMA)
BCM2835_DEFINE_BLOCK_TYPE(Denoise, denoise, DENOISE)
BCM2835_DEFINE_BLOCK_TYPE(Sharpen, sharpen, SHARPEN)
BCM2835_DEFINE_BLOCK_TYPE(Dpc, dpc, DPC)
BCM2835_DEFINE_BLOCK_TYPE(Cdn, cdn, CDN)
BCM2835_DEFINE_BLOCK_TYPE(CcMatrix, cc_matrix, CC_MATRIX)
BCM2835_DEFINE_BLOCK_TYPE(LensShading, lens_shading, LENS_SHADING)
BCM2835_DEFINE_BLOCK_TYPE(AwbGains, awb_gains, AWB_GAINS)
BCM2835_DEFINE_BLOCK_TYPE(DGain, digital_gain, DIGITAL_GAIN)

struct params_traits {
	using id_type = BlockType;
	template<id_type Id>
	using id_to_details = block_type<Id>;
};

} /* namespace details */

class Bcm2835Params : public V4L2Params<details::params_traits>
{
public:
	Bcm2835Params(Span<uint8_t> data) : V4L2Params(data,
						       BCM2835_ISP_PARAM_BUFFER_V1)
	{
	}
};

} /* namespace ipa::RPi */

} /* namespace libcamera */
