/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: e:\niu\Flow\src\modules\uavcannode\dsdl\uavcan\threedr\equipment\flow\optical_flow\1110.LegacyRawSample.uavcan
 */

#ifndef THREEDR_EQUIPMENT_FLOW_OPTICAL_FLOW_LEGACYRAWSAMPLE_HPP_INCLUDED
#define THREEDR_EQUIPMENT_FLOW_OPTICAL_FLOW_LEGACYRAWSAMPLE_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

#include <threedr\equipment\flow\optical_flow\I2CFrame.hpp>
#include <threedr\equipment\flow\optical_flow\I2CFrameIntegral.hpp>
#include <uavcan\Timestamp.hpp>

/******************************* Source text **********************************
# Current time.
# Note that the data type "uavcan.Timestamp" is defined by the UAVCAN specification.
uavcan.Timestamp time

I2CFrame frame
I2CFrameIntegral integral
******************************************************************************/

/********************* DSDL signature source definition ***********************
threedr.equipment.flow.optical_flow.LegacyRawSample
uavcan.Timestamp time
threedr.equipment.flow.optical_flow.I2CFrame frame
threedr.equipment.flow.optical_flow.I2CFrameIntegral integral
******************************************************************************/

#undef time
#undef frame
#undef integral

namespace threedr
{
namespace equipment
{
namespace flow
{
namespace optical_flow
{

template <int _tmpl>
struct UAVCAN_EXPORT LegacyRawSample_
{
    typedef const LegacyRawSample_<_tmpl>& ParameterType;
    typedef LegacyRawSample_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::Timestamp time;
        typedef ::threedr::equipment::flow::optical_flow::I2CFrame frame;
        typedef ::threedr::equipment::flow::optical_flow::I2CFrameIntegral integral;
    };

    enum
    {
        MinBitLen
            = FieldTypes::time::MinBitLen
            + FieldTypes::frame::MinBitLen
            + FieldTypes::integral::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::time::MaxBitLen
            + FieldTypes::frame::MaxBitLen
            + FieldTypes::integral::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::time >::Type time;
    typename ::uavcan::StorageType< typename FieldTypes::frame >::Type frame;
    typename ::uavcan::StorageType< typename FieldTypes::integral >::Type integral;

    LegacyRawSample_()
        : time()
        , frame()
        , integral()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<432 == MaxBitLen>::check();
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
    enum { DefaultDataTypeID = 1110 };

    static const char* getDataTypeFullName()
    {
        return "threedr.equipment.flow.optical_flow.LegacyRawSample";
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
bool LegacyRawSample_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        time == rhs.time &&
        frame == rhs.frame &&
        integral == rhs.integral;
}

template <int _tmpl>
bool LegacyRawSample_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(time, rhs.time) &&
        ::uavcan::areClose(frame, rhs.frame) &&
        ::uavcan::areClose(integral, rhs.integral);
}

template <int _tmpl>
int LegacyRawSample_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::time::encode(self.time, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::frame::encode(self.frame, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::integral::encode(self.integral, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int LegacyRawSample_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::time::decode(self.time, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::frame::decode(self.frame, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::integral::decode(self.integral, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature LegacyRawSample_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x2F715558D687E221ULL);

    FieldTypes::time::extendDataTypeSignature(signature);
    FieldTypes::frame::extendDataTypeSignature(signature);
    FieldTypes::integral::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef LegacyRawSample_<0> LegacyRawSample;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::threedr::equipment::flow::optical_flow::LegacyRawSample > _uavcan_gdtr_registrator_LegacyRawSample;

}

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
class UAVCAN_EXPORT YamlStreamer< ::threedr::equipment::flow::optical_flow::LegacyRawSample >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::threedr::equipment::flow::optical_flow::LegacyRawSample::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::threedr::equipment::flow::optical_flow::LegacyRawSample >::stream(Stream& s, ::threedr::equipment::flow::optical_flow::LegacyRawSample::ParameterType obj, const int level)
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
    s << "time: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::LegacyRawSample::FieldTypes::time >::stream(s, obj.time, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "frame: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::LegacyRawSample::FieldTypes::frame >::stream(s, obj.frame, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "integral: ";
    YamlStreamer< ::threedr::equipment::flow::optical_flow::LegacyRawSample::FieldTypes::integral >::stream(s, obj.integral, level + 1);
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
inline Stream& operator<<(Stream& s, ::threedr::equipment::flow::optical_flow::LegacyRawSample::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::threedr::equipment::flow::optical_flow::LegacyRawSample >::stream(s, obj, 0);
    return s;
}

} // Namespace optical_flow
} // Namespace flow
} // Namespace equipment
} // Namespace threedr

#endif // THREEDR_EQUIPMENT_FLOW_OPTICAL_FLOW_LEGACYRAWSAMPLE_HPP_INCLUDED