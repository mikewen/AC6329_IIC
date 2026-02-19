#include "api/sensor.h"
#include "adaptation.h"
#include "api/properties.h"

enum scalar_repr_flags {
    UNSIGNED = 0,
    SIGNED = BIT(1),
    DIVIDE = BIT(2),
    /** The highest encoded value represents "undefined" */
    HAS_UNDEFINED = BIT(3),
    /**
     * The highest encoded value represents
     * the maximum value or higher.
     */
    HAS_HIGHER_THAN = (BIT(4)),
    /** The second highest encoded value represents "value is invalid" */
    HAS_INVALID = (BIT(5)),
    HAS_MAX = BIT(6),
    HAS_MIN = BIT(7),
    HAS_UNDEFINED_MIN = BIT(8),
};

/** @brief 64-bit unsigned integer with bit position @p _n set. */
#define BIT64(_n) (1ULL << (_n))

struct scalar_repr {
    enum scalar_repr_flags flags;
    int32_t min;
    uint32_t max; /**< Highest encoded value */
    int64_t value;
};

static int64_t scalar_max(const struct bt_mesh_sensor_format *format)
{
    const struct scalar_repr *repr = format->user_data;

    if (repr->flags & HAS_MAX) {
        return repr->max;
    }

    int64_t max_value = BIT64(8 * format->size) - 1;

    if (repr->flags & SIGNED) {
        max_value = BIT64(8 * format->size - 1) - 1;
    }

    if (repr->flags & HAS_INVALID) {
        max_value -= 2;
    } else if (repr->flags & HAS_UNDEFINED) {
        max_value -= 1;
    }

    return max_value;
}

static int32_t scalar_min(const struct bt_mesh_sensor_format *format)
{
    const struct scalar_repr *repr = format->user_data;

    if (repr->flags & HAS_MIN) {
        return repr->min;
    }

    if (repr->flags & SIGNED) {
        if (repr->flags & HAS_UNDEFINED_MIN) {
            return -BIT64(8 * format->size - 1) + 1;
        }
        return -BIT64(8 * format->size - 1);
    }

    return 0;
}

static int64_t mul_scalar(int64_t val, const struct scalar_repr *repr)
{
    return (repr->flags & DIVIDE) ? (val / repr->value) :
           (val * repr->value);
}

static int64_t div_scalar(int64_t val, const struct scalar_repr *repr)
{
    return (repr->flags & DIVIDE) ? (val * repr->value) :
           (val / repr->value);
}


static int scalar_decode(const struct bt_mesh_sensor_format *format,
                         struct net_buf_simple *buf, struct sensor_value *val)
{
    const struct scalar_repr *repr = format->user_data;

    if (buf->len < format->size) {
        return -ENOMEM;
    }

    int64_t raw;

    switch (format->size) {
    case 1:
        if (repr->flags & SIGNED) {
            raw = (int8_t) net_buf_simple_pull_u8(buf);
        } else {
            raw = net_buf_simple_pull_u8(buf);
        }
        break;
    case 2:
        if (repr->flags & SIGNED) {
            raw = (int16_t) net_buf_simple_pull_le16(buf);
        } else {
            raw = net_buf_simple_pull_le16(buf);
        }
        break;
    case 3:
        raw = net_buf_simple_pull_le24(buf);

        if ((repr->flags & SIGNED) && (raw & BIT(24))) {
            raw |= (BIT_MASK(8) << 24);
        }
        break;
    case 4:
        raw = net_buf_simple_pull_le32(buf);
        break;
    default:
        return -ERANGE;
    }

    int64_t max_value = scalar_max(format);
    int64_t min_value = scalar_min(format);

    if (raw < min_value || raw > max_value) {
        if (!(repr->flags & (HAS_UNDEFINED | HAS_UNDEFINED_MIN |
                             HAS_HIGHER_THAN | HAS_INVALID))) {
            return -ERANGE;
        }

        uint32_t type_max = (repr->flags & SIGNED) ?
                            BIT64(8 * format->size - 1) - 1 :
                            BIT64(8 * format->size) - 1;

        if (repr->flags & HAS_UNDEFINED_MIN) {
            type_max += 1;
        } else if ((repr->flags & HAS_INVALID) && raw == type_max - 1) {
            type_max -= 1;
        } else if (raw != type_max) {
            return -ERANGE;
        }

        val->val1 = type_max;
        val->val2 = 0;
        return 0;
    }

    int64_t million = mul_scalar(raw * 1000000LL, repr);

    val->val1 = million / 1000000LL;
    val->val2 = million % 1000000LL;

    return 0;
}

static int scalar_encode(const struct bt_mesh_sensor_format *format,
                         const struct sensor_value *val,
                         struct net_buf_simple *buf)
{
    const struct scalar_repr *repr = format->user_data;

    if (net_buf_simple_tailroom(buf) < format->size) {
        return -ENOMEM;
    }

    int64_t raw = div_scalar(val->val1, repr) +
                  div_scalar(val->val2, repr) / 1000000LL;

    int64_t max_value = scalar_max(format);
    int32_t min_value = scalar_min(format);

    if (raw > max_value || raw < min_value) {
        uint32_t type_max = BIT64(8 * format->size) - 1;
        if (repr->flags & SIGNED) {
            type_max = BIT64(8 * format->size - 1) - 1;
        }

        if ((repr->flags & HAS_HIGHER_THAN) &&
            !((repr->flags & HAS_UNDEFINED) && (val->val1 == type_max))) {
            raw = max_value;
        } else if ((repr->flags & HAS_INVALID) && val->val1 == type_max - 1) {
            raw = type_max - 1;
        } else if ((repr->flags & HAS_UNDEFINED) && val->val1 == type_max) {
            raw = type_max;
        } else if (repr->flags & HAS_UNDEFINED_MIN) {
            raw = type_max + 1;
        } else {
            return -ERANGE;
        }
    }

    switch (format->size) {
    case 1:
        net_buf_simple_add_u8(buf, raw);
        break;
    case 2:
        net_buf_simple_add_le16(buf, raw);
        break;
    case 3:
        net_buf_simple_add_le24(buf, raw);
        break;
    case 4:
        net_buf_simple_add_le32(buf, raw);
        break;
    default:
        return -EIO;
    }

    return 0;
}

static int float32_encode(const struct bt_mesh_sensor_format *format,
                          const struct sensor_value *val,
                          struct net_buf_simple *buf)
{
    if (net_buf_simple_tailroom(buf) < format->size) {
        return -ENOMEM;
    }

    /* IEEE-754 32-bit floating point */
    float fvalue = (float)val->val1 + (float)val->val2 / 1000000L;

    net_buf_simple_add_mem(buf, &fvalue, sizeof(float));

    return 0;
}

static int float32_decode(const struct bt_mesh_sensor_format *format,
                          struct net_buf_simple *buf, struct sensor_value *val)
{
    if (buf->len < format->size) {
        return -ENOMEM;
    }

    float fvalue;

    memcpy(&fvalue, net_buf_simple_pull_mem(buf, sizeof(float)),
           sizeof(float));

    val->val1 = (int32_t)fvalue;
    val->val2 = ((int64_t)(fvalue * 1000000.0f)) % 1000000L;
    return 0;
}

static int boolean_encode(const struct bt_mesh_sensor_format *format,
                          const struct sensor_value *val,
                          struct net_buf_simple *buf)
{
    if (net_buf_simple_tailroom(buf) < 1) {
        return -ENOMEM;
    }

    net_buf_simple_add_u8(buf, !!val->val1);
    return 0;
}

static int boolean_decode(const struct bt_mesh_sensor_format *format,
                          struct net_buf_simple *buf, struct sensor_value *val)
{
    if (buf->len < 1) {
        return -ENOMEM;
    }

    uint8_t b = net_buf_simple_pull_u8(buf);

    if (b > 1) {
        return -EINVAL;
    }

    val->val1 = b;
    val->val2 = 0;
    return 0;
}

#define UNIT(name) const struct bt_mesh_sensor_unit bt_mesh_sensor_unit_##name

#define CHANNELS(...)                                                          \
	.channels = ((const struct bt_mesh_sensor_channel[]){ __VA_ARGS__ }),  \
	.channel_count = ARRAY_SIZE(                                           \
		((const struct bt_mesh_sensor_channel[]){ __VA_ARGS__ }))

#define SCALAR_IS_DIV(_scalar) ((_scalar) > -1.0 && (_scalar) < 1.0)

#define SCALAR_REPR_RANGED(_scalar, _flags, _min, _max)                              \
	{                                                                      \
		.flags = ((_flags) | (SCALAR_IS_DIV(_scalar) ? DIVIDE : 0)),   \
		.min = _min,                                                   \
		.max = _max,                                                   \
		.value = (int64_t)((SCALAR_IS_DIV(_scalar) ? (1.0 / (_scalar)) : \
							   (_scalar)) +        \
				 0.5),                                         \
	}

#define SCALAR_REPR(_scalar, _flags) SCALAR_REPR_RANGED(_scalar, _flags, 0, 0)

#define SENSOR_TYPE(name)                                                      \
	const STRUCT_SECTION_ITERABLE(bt_mesh_sensor_type,                     \
					bt_mesh_sensor_##name)

#define CHANNEL(_name, _format)                                                \
	{                                                                      \
		.format = &bt_mesh_sensor_format_##_format,                    \
	}

#define SCALAR_FORMAT(_size, _flags, _unit, _scalar)                           \
	{                                                                      \
		.encode = scalar_encode, .decode = scalar_decode,              \
		.size = _size,                                                 \
		.user_data = (void *)&((const struct scalar_repr)SCALAR_REPR(  \
			_scalar, _flags)),                                     \
	}

#define SCALAR_FORMAT_MAX(_size, _flags, _unit, _scalar, _max)                 \
	{                                                                      \
		.encode = scalar_encode, .decode = scalar_decode,              \
		.size = _size,                                                 \
		.user_data = (void *)&(                                        \
			(const struct scalar_repr)SCALAR_REPR_RANGED(          \
				_scalar, ((_flags) | HAS_MAX), 0, _max)),         \
	}

#define FORMAT(_name)                                                          \
	const struct bt_mesh_sensor_format bt_mesh_sensor_format_##_name

/*******************************************************************************
 * Units
 ******************************************************************************/
UNIT(lux) = { "Lux", "lx" };
UNIT(kwh) = { "Kilo Watt hours", "kWh" };
UNIT(seconds) = { "Seconds", "s" };
UNIT(unitless) = { "Unitless" };
/*******************************************************************************
 * Encoders and decoders
 ******************************************************************************/
#define BRANCHLESS_ABS8(x) (((x) + ((x) >> 7)) ^ ((x) >> 7))

#define SCALAR(mul, bin)                                                       \
	((double)(mul) *                                                       \
	 (double)((bin) >= 0 ? (1ULL << BRANCHLESS_ABS8(bin)) :                \
			       (1.0 / (1ULL << BRANCHLESS_ABS8(bin)))))

FORMAT(illuminance)		= SCALAR_FORMAT(3,
                                        (UNSIGNED | HAS_UNDEFINED),
                                        lux,
                                        SCALAR(1e-2, 0));

FORMAT(energy32)	 = SCALAR_FORMAT(4,
                                     UNSIGNED | HAS_INVALID | HAS_UNDEFINED,
                                     kwh,
                                     SCALAR(1e-3, 0));

FORMAT(time_second_16) = SCALAR_FORMAT(2,
                                       (UNSIGNED | HAS_UNDEFINED),
                                       seconds,
                                       SCALAR(1, 0));

FORMAT(count_16)	 = SCALAR_FORMAT(2,
                                     (UNSIGNED | HAS_UNDEFINED),
                                     unitless,
                                     SCALAR(1, 0));

FORMAT(percentage_8)  = SCALAR_FORMAT_MAX(1,
                        (UNSIGNED | HAS_UNDEFINED),
                        percent,
                        SCALAR(1, -1),
                        200);

FORMAT(perceived_lightness)	= SCALAR_FORMAT(2,
                              UNSIGNED,
                              unitless,
                              SCALAR(1, 0));

FORMAT(time_millisecond_24) = SCALAR_FORMAT(3,
                              (UNSIGNED | HAS_UNDEFINED),
                              seconds,
                              SCALAR(1e-3, 0));

FORMAT(boolean) = {
    .encode = boolean_encode,
    .decode = boolean_decode,
    .size	= 1,
#ifdef CONFIG_BT_MESH_SENSOR_LABELS
    .unit = &bt_mesh_sensor_unit_unitless,
#endif
};

FORMAT(coefficient) = {
    .encode = float32_encode,
    .decode = float32_decode,
    .size	= 4,
#ifdef CONFIG_BT_MESH_SENSOR_LABELS
    .unit = &bt_mesh_sensor_unit_unitless,
#endif
};

const struct bt_mesh_sensor_type bt_mesh_sensor_gain = {
    .id = BT_MESH_PROP_ID_SENSOR_GAIN,
    CHANNELS(CHANNEL("Sensor gain", coefficient)),
};

const struct bt_mesh_sensor_type bt_mesh_sensor_present_amb_light_level = {
    .id = BT_MESH_PROP_ID_PRESENT_AMB_LIGHT_LEVEL,
    CHANNELS(CHANNEL("Present ambient light level", illuminance)),
};

const struct bt_mesh_sensor_type bt_mesh_sensor_presence_detected = {
    .id = BT_MESH_PROP_ID_PRESENCE_DETECTED,
    CHANNELS(CHANNEL("Presence detected", boolean)),
};

const struct bt_mesh_sensor_type bt_mesh_sensor_time_since_presence_detected = {
    .id = BT_MESH_PROP_ID_TIME_SINCE_PRESENCE_DETECTED,
    CHANNELS(CHANNEL("Time since presence detected", time_second_16)),
};

const struct bt_mesh_sensor_type bt_mesh_sensor_motion_threshold = {
    .id = BT_MESH_PROP_ID_MOTION_THRESHOLD,
    CHANNELS(CHANNEL("Motion threshold", percentage_8)),
};

const struct bt_mesh_sensor_type bt_mesh_sensor_motion_sensed = {
    .id = BT_MESH_PROP_ID_MOTION_SENSED,
    CHANNELS(CHANNEL("Motion sensed", percentage_8)),
};

const struct bt_mesh_sensor_type bt_mesh_sensor_people_count = {
    .id = BT_MESH_PROP_ID_PEOPLE_COUNT,
    CHANNELS(CHANNEL("People count", count_16)),
};

const struct bt_mesh_sensor_type bt_mesh_sensor_time_since_motion_sensed = {
    .id = BT_MESH_PROP_ID_TIME_SINCE_MOTION_SENSED,
    CHANNELS(CHANNEL("Time since motion detected", time_second_16)),
};

const struct bt_mesh_sensor_type bt_mesh_sensor_precise_tot_dev_energy_use = {
    .id = BT_MESH_PROP_ID_PRECISE_TOT_DEV_ENERGY_USE,
    CHANNELS(CHANNEL("Total device energy use", energy32)),
};

// sensor list
const struct bt_mesh_sensor_type *const sensor_type_list[] = {
    &bt_mesh_sensor_gain,
    &bt_mesh_sensor_precise_tot_dev_energy_use,
    &bt_mesh_sensor_present_amb_light_level,
    &bt_mesh_sensor_presence_detected,
    &bt_mesh_sensor_time_since_presence_detected,
    &bt_mesh_sensor_motion_threshold,
};
