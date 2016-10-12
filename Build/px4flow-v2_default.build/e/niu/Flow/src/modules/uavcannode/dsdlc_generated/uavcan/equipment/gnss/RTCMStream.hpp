/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: e:\niu\Flow\src\lib\uavcan\dsdl\uavcan\equipment\gnss\1062.RTCMStream.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# GNSS RTCM SC-104 protocol raw stream container.
# RTCM messages that are longer than max data size can be split over multiple consecutive messages.
#

uint8 PROTOCOL_ID_UNKNOWN = 0
uint8 PROTOCOL_ID_RTCM2   = 2
uint8 PROTOCOL_ID_RTCM3   = 3
uint8 protocol_id

uint8[<=128] data
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.gnss.RTCMStream
saturated uint8 protocol_id
saturated uint8[<=128] data
******************************************************************************/

#undef protocol_id
#undef data
#undef PROTOCOL_ID_UNKNOWN
#undef PROTOCOL_ID_RTCM2
#undef PROTOCOL_ID_RTCM3

namespace uavcan
{
namespace equipment
{
namespace gnss
{

template <int _tmpl>
struct UAVCAN_EXPORT RTCMStream_
{
    typedef const RTCMStream_<_tmpl>& ParameterType;
    typedef RTCMStream_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > PROTOCOL_ID_UNKNOWN;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > PROTOCOL_ID_RTCM2;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > PROTOCOL_ID_RTCM3;
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > protocol_id;
        typedef ::uavcan::Array< ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeDynamic, 128 > data;
    };

    enum
    {
        MinBitLen
            = FieldTypes::protocol_id::MinBitLen
            + FieldTypes::data::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::protocol_id::MaxBitLen
            + FieldTypes::data::MaxBitLen
    };

    // Constants
    static const typename ::uavcan::StorageType< typename ConstantTypes::PROTOCOL_ID_UNKNOWN >::Type PROTOCOL_ID_UNKNOWN; // 0
    static const typename ::uavcan::StorageType< typename ConstantTypes::PROTOCOL_ID_RTCM2 >::Type PROTOCOL_ID_RTCM2; // 2
    static const typename ::uavcan::StorageType< typename ConstantTypes::PROTOCOL_ID_RTCM3 >::Type PROTOCOL_ID_RTCM3; // 3

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::protocol_id >::Type protocol_id;
    typename ::uavcan::StorageType< typename FieldTypes::data >::Type data;

    RTCMStream_()
        : protocol_id()
        , data()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<1040 == MaxBitLen>::check();
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
    enum { DefaultDataTypeID = 1062 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.equipment.gnss.RTCMStream";
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
bool RTCMStream_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        protocol_id == rhs.protocol_id &&
        data == rhs.data;
}

template <int _tmpl>
bool RTCMStream_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(protocol_id, rhs.protocol_id) &&
        ::uavcan::areClose(data, rhs.data);
}

template <int _tmpl>
int RTCMStream_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::protocol_id::encode(self.protocol_id, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::data::encode(self.data, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int RTCMStream_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::protocol_id::decode(self.protocol_id, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::data::decode(self.data, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature RTCMStream_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x1F56030ECB171501ULL);

    FieldTypes::protocol_id::extendDataTypeSignature(signature);
    FieldTypes::data::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

template <int _tmpl>
const typename ::uavcan::StorageType< typename RTCMStream_<_tmpl>::ConstantTypes::PROTOCOL_ID_UNKNOWN >::Type
    RTCMStream_<_tmpl>::PROTOCOL_ID_UNKNOWN = 0U; // 0

template <int _tmpl>
const typename ::uavcan::StorageType< typename RTCMStream_<_tmpl>::ConstantTypes::PROTOCOL_ID_RTCM2 >::Type
    RTCMStream_<_tmpl>::PROTOCOL_ID_RTCM2 = 2U; // 2

template <int _tmpl>
const typename ::uavcan::StorageType< typename RTCMStream_<_tmpl>::ConstantTypes::PROTOCOL_ID_RTCM3 >::Type
    RTCMStream_<_tmpl>::PROTOCOL_ID_RTCM3 = 3U; // 3

/*
 * Final typedef
 */
typedef RTCMStream_<0> RTCMStream;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::equipment::gnss::RTCMStream > _uavcan_gdtr_registrator_RTCMStream;

}

} // Namespace gnss
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::gnss::RTCMStream >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::gnss::RTCMStream::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::gnss::RTCMStream >::stream(Stream& s, ::uavcan::equipment::gnss::RTCMStream::ParameterType obj, const int level)
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
    s << "protocol_id: ";
    YamlStreamer< ::uavcan::equipment::gnss::RTCMStream::FieldTypes::protocol_id >::stream(s, obj.protocol_id, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "data: ";
    YamlStreamer< ::uavcan::equipment::gnss::RTCMStream::FieldTypes::data >::stream(s, obj.data, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace gnss
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::gnss::RTCMStream::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::gnss::RTCMStream >::stream(s, obj, 0);
    return s;
}

} // Namespace gnss
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_HPP_INCLUDED