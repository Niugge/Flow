/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: e:\niu\Flow\src\modules\uavcannode\dsdl\uavcan\threedr\equipment\flow\optical_flow\I2CFrame.uavcan
 */

#ifndef THREEDR_EQUIPMENT_FLOW_OPTICAL_FLOW_I2CFRAME_HPP_INCLUDED
#define THREEDR_EQUIPMENT_FLOW_OPTICAL_FLOW_I2CFRAME_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Nested Type
# Legacy I2C
#

uint16 frame_count
int16 pixel_flow_x_sum
int16 pixel_flow_y_sum
int16 flow_comp_m_x
int16 flow_comp_m_y
int16 qual
int16 gyro_x_rate
int16 gyro_y_rate
int16 gyro_z_rate
uint8 gyro_range
uint8 sonarimestamp
int16 ground_distance
******************************************************************************/

/********************* DSDL signature source definition ***********************
threedr.equipment.flow.optical_flow.I2CFrame
saturated uint16 frame_count
saturated int16 pixel_flow_x_sum
saturated int16 pixel_flow_y_sum
saturated int16 flow_comp_m_x
saturated int16 flow_comp_m_y
saturated int16 qual
saturated int16 gyro_x_rate
saturated int16 gyro_y_rate
saturated int16 gyro_z_rate
saturated uint8 gyro_range
saturated uint8 sonarimestamp
saturated int16 ground_distance
******************************************************************************/

#undef frame_count
#undef pixel_flow_x_sum
#undef pixel_flow_y_sum
#undef flow_comp_m_x
#undef flow_comp_m_y
#undef qual
#undef gyro_x_rate
#undef gyro_y_rate
#undef gyro_z_rate
#undef gyro_range
#undef sonarimestamp
#undef ground_distance

namespace threedr
{
namespace equipment
{
namespace flow
{
namespace optical_flow
{

template <int _tmpl>
struct UAVCAN_EXPORT I2CFrame_
{
    typedef const I2CFrame_<_tmpl>& ParameterType;
    typedef I2CFrame_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > frame_count;
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > pixel_flow_x_sum;
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > pixel_flow_y_sum;
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > flow_comp_m_x;
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > flow_comp_m_y;
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > qual;
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > gyro_x_rate;
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > gyro_y_rate;
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > gyro_z_rate;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > gyro_range;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > sonarimestamp;
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > ground_distance;
    };

    enum
    {
        MinBitLen
            = FieldTypes::frame_count::MinBitLen
            + FieldTypes::pixel_flow_x_sum::MinBitLen
            + FieldTypes::pixel_flow_y_sum::MinBitLen
            + FieldTypes::flow_comp_m_x::MinBitLen
            + FieldTypes::flow_comp_m_y::MinBitLen
            + FieldTypes::qual::MinBitLen
            + FieldTypes::gyro_x_rate::MinBitLen
            + FieldTypes::gyro_y_rate::MinBitLen
            + FieldTypes::gyro_z_rate::MinBitLen
            + FieldTypes::gyro_range::MinBitLen
            + FieldTypes::sonarimestamp::MinBitLen
            + FieldTypes::ground_distance::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::frame_count::MaxBitLen
            + FieldTypes::pixel_flow_x_sum::MaxBitLen
            + FieldTypes::pixel_flow_y_sum::MaxBitLen
            + FieldTypes::flow_comp_m_x::MaxBitLen
            + FieldTypes::flow_comp_m_y::MaxBitLen
            + FieldTypes::qual::MaxBitLen
            + FieldTypes::gyro_x_rate::MaxBitLen
            + FieldTypes::gyro_y_rate::MaxBitLen
            + FieldTypes::gyro_z_rate::MaxBitLen
            + FieldTypes::gyro_range::MaxBitLen
            + FieldTypes::sonarimestamp::MaxBitLen
            + FieldTypes::ground_distance::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::frame_count >::Type frame_count;
    typename ::uavcan::StorageType< typename FieldTypes::pixel_flow_x_sum >::Type pixel_flow_x_sum;
    typename ::uavcan::StorageType< typename FieldTypes::pixel_flow_y_sum >::Type pixel_flow_y_sum;
    typename ::uavcan::StorageType< typename FieldTypes::flow_comp_m_x >::Type flow_comp_m_x;
    typename ::uavcan::StorageType< typename FieldTypes::flow_comp_m_y >::Type flow_comp_m_y;
    typename ::uavcan::StorageType< typename FieldTypes::qual >::Type qual;
    typename ::uavcan::StorageType< typename FieldTypes::gyro_x_rate >::Type gyro_x_rate;
    typename ::uavcan::StorageType< typename FieldTypes::gyro_y_rate >::Type gyro_y_rate;
    typename ::uavcan::StorageType< typename FieldTypes::gyro_z_rate >::Type gyro_z_rate;
    typename ::uavcan::StorageType< typename FieldTypes::gyro_range >::Type gyro_range;
    typename ::uavcan::StorageType< typename FieldTypes::sonarimestamp >::Type sonarimestamp;
    typename ::uavcan::StorageType< typename FieldTypes::ground_distance >::Type ground_distance;

    I2CFrame_()
        : frame_count()
        , pixel_flow_x_sum()
        , pixel_flow_y_sum()
        , flow_comp_m_x()
        , flow_comp_m_y()
        , qual()
        , gyro_x_rate()
        , gyro_y_rate()
        , gyro_z_rate()
        , gyro_range()
        , sonarimestamp()
        , ground_distance()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<176 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    // This type has no default data type ID

    static const char* getDataTypeFullName()
    {
        return "threedr.equipment.flow.optical_flow.I2CFrame";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool I2CFrame_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        frame_count == rhs.frame_count &&
        pixel_flow_x_sum == rhs.pixel_flow_x_sum &&
        pixel_flow_y_sum == rhs.pixel_flow_y_sum &&
        flow_comp_m_x == rhs.flow_comp_m_x &&
        flow_comp_m_y == rhs.flow_comp_m_y &&
        qual == rhs.qual &&
        gyro_x_rate == rhs.gyro_x_rate &&
        gyro_y_rate == rhs.gyro_y_rate &&
        gyro_z_rate == rhs.gyro_z_rate &&
        gyro_range == rhs.gyro_range &&
        sonarimestamp == rhs.sonarimestamp &&
        ground_distance == rhs.ground_distance;
}

template <int _tmpl>
bool I2CFrame_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(frame_count, rhs.frame_count) &&
        ::uavcan::areClose(pixel_flow_x_sum, rhs.pixel_flow_x_sum) &&
        ::uavcan::areClose(pixel_flow_y_sum, rhs.pixel_flow_y_sum) &&
        ::uavcan::areClose(flow_comp_m_x, rhs.flow_comp_m_x) &&
        ::uavcan::areClose(flow_comp_m_y, rhs.flow_comp_m_y) &&
        ::uavcan::areClose(qual, rhs.qual) &&
        ::uavcan::areClose(gyro_x_rate, rhs.gyro_x_rate) &&
        ::uavcan::areClose(gyro_y_rate, rhs.gyro_y_rate) &&
        ::uavcan::areClose(gyro_z_rate, rhs.gyro_z_rate) &&
        ::uavcan::areClose(gyro_range, rhs.gyro_range) &&
        ::uavcan::areClose(sonarimestamp, rhs.sonarimestamp) &&
        ::uavcan::areClose(ground_distance, rhs.ground_distance);
}

template <int _tmpl>
int I2CFrame_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::frame_count::encode(self.frame_count, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::pixel_flow_x_sum::encode(self.pixel_flow_x_sum, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::pixel_flow_y_sum::encode(self.pixel_flow_y_sum, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::flow_comp_m_x::encode(self.flow_comp_m_x, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::flow_comp_m_y::encode(self.flow_comp_m_y, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::qual::encode(self.qual, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::gyro_x_rate::encode(self.gyro_x_rate, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::gyro_y_rate::encode(self.gyro_y_rate, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::gyro_z_rate::encode(self.gyro_z_rate, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::gyro_range::encode(self.gyro_range, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::sonarimestamp::encode(self.sonarimestamp, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::ground_distance::encode(self.ground_distance, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int I2CFrame_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::frame_count::decode(self.frame_count, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::pixel_flow_x_sum::decode(self.pixel_flow_x_sum, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::pixel_flow_y_sum::decode(self.pixel_flow_y_sum, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::flow_comp_m_x::decode(self.flow_comp_m_x, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::flow_comp_m_y::decode(self.flow_comp_m_y, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::qual::decode(self.qual, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::gyro_x_rate::decode(self.gyro_x_rate, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::gyro_y_rate::decode(self.gyro_y_rate, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::gyro_z_rate::decode(self.gyro_z_rate, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::gyro_range::decode(self.gyro_range, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::sonarimestamp::decode(self.sonarimestamp, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::ground_distance::decode(self.ground_distance, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature I2CFrame_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xD4CF63140C693827ULL);

    FieldTypes::frame_count::extendDataTypeSignature(signature);
    FieldTypes::pixel_flow_x_sum::extendDataTypeSignature(signature);
    FieldTypes::pixel_flow_y_sum::extendDataTypeSignature(signature);
    FieldTypes::flow_comp_m_x::extendDataTypeSignature(signature);
    FieldTypes::flow_comp_m_y::extendDataTypeSignature(signature);
    FieldTypes::qual::extendDataTypeSignature(signature);
    FieldTypes::gyro_x_rate::extendDataTypeSignature(signature);
    FieldTypes::gyro_y_rate::extendDataTypeSignature(signature);
    FieldTypes::gyro_z_rate::extendDataTypeSignature(signature);
    FieldTypes::gyro_range::extendDataTypeSignature(signature);
    FieldTypes::sonarimestamp::extendDataTypeSignature(signature);
    FieldTypes::ground_distance::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef I2CFrame_<0> I2CFrame;

// No default registration

} // Namespace optical_flow
} // Namespace flow
} // Namespace equipment
} // Namespace threedr

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrame >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::threedr::equipment::flow::optical_flow::I2CFrame::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrame >::stream(Stream& s, ::threedr::equipment::flow::optical_flow::I2CFrame::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "frame_count: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrame::FieldTypes::frame_count >::stream(s, obj.frame_count, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "pixel_flow_x_sum: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrame::FieldTypes::pixel_flow_x_sum >::stream(s, obj.pixel_flow_x_sum, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "pixel_flow_y_sum: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrame::FieldTypes::pixel_flow_y_sum >::stream(s, obj.pixel_flow_y_sum, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "flow_comp_m_x: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrame::FieldTypes::flow_comp_m_x >::stream(s, obj.flow_comp_m_x, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "flow_comp_m_y: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrame::FieldTypes::flow_comp_m_y >::stream(s, obj.flow_comp_m_y, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "qual: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrame::FieldTypes::qual >::stream(s, obj.qual, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "gyro_x_rate: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrame::FieldTypes::gyro_x_rate >::stream(s, obj.gyro_x_rate, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "gyro_y_rate: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrame::FieldTypes::gyro_y_rate >::stream(s, obj.gyro_y_rate, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "gyro_z_rate: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrame::FieldTypes::gyro_z_rate >::stream(s, obj.gyro_z_rate, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "gyro_range: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrame::FieldTypes::gyro_range >::stream(s, obj.gyro_range, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "sonarimestamp: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrame::FieldTypes::sonarimestamp >::stream(s, obj.sonarimestamp, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "ground_distance: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrame::FieldTypes::ground_distance >::stream(s, obj.ground_distance, level + 1);
}

}

namespace threedr
{
namespace equipment
{
namespace flow
{
namespace optical_flow
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::threedr::equipment::flow::optical_flow::I2CFrame::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::threedr::equipment::flow::optical_flow::I2CFrame >::stream(s, obj, 0);
    return s;
}

} // Namespace optical_flow
} // Namespace flow
} // Namespace equipment
} // Namespace threedr

#endif // THREEDR_EQUIPMENT_FLOW_OPTICAL_FLOW_I2CFRAME_HPP_INCLUDED