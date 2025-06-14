/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#ifndef __lcmmessages_dat_driveline_msg_hpp__
#define __lcmmessages_dat_driveline_msg_hpp__

#include <lcm/lcm_coretypes.h>

#include <vector>
#include "lcmmessages/header.hpp"
#include "lcmmessages/MotorTelemetry.hpp"

namespace lcmmessages
{

class dat_driveline_msg
{
    public:
        lcmmessages::header hdr;

        int16_t    num_motors;

        std::vector< lcmmessages::MotorTelemetry > motor_data;

        float      fwd_speed;

        float      yaw_rate;

        int8_t     is_emergency_stop;

        int8_t     is_external_control;

        int32_t    odometer;

        int32_t    time_online;

    public:
        /**
         * Encode a message into binary form.
         *
         * @param buf The output buffer.
         * @param offset Encoding starts at thie byte offset into @p buf.
         * @param maxlen Maximum number of bytes to write.  This should generally be
         *  equal to getEncodedSize().
         * @return The number of bytes encoded, or <0 on error.
         */
        inline int encode(void *buf, int offset, int maxlen) const;

        /**
         * Check how many bytes are required to encode this message.
         */
        inline int getEncodedSize() const;

        /**
         * Decode a message from binary form into this instance.
         *
         * @param buf The buffer containing the encoded message.
         * @param offset The byte offset into @p buf where the encoded message starts.
         * @param maxlen The maximum number of bytes to read while decoding.
         * @return The number of bytes decoded, or <0 if an error occured.
         */
        inline int decode(const void *buf, int offset, int maxlen);

        /**
         * Retrieve the 64-bit fingerprint identifying the structure of the message.
         * Note that the fingerprint is the same for all instances of the same
         * message type, and is a fingerprint on the message type definition, not on
         * the message contents.
         */
        inline static int64_t getHash();

        /**
         * Returns "dat_driveline_msg"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int dat_driveline_msg::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int dat_driveline_msg::decode(const void *buf, int offset, int maxlen)
{
    int pos = 0, thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int dat_driveline_msg::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t dat_driveline_msg::getHash()
{
    static int64_t hash = static_cast<int64_t>(_computeHash(NULL));
    return hash;
}

const char* dat_driveline_msg::getTypeName()
{
    return "dat_driveline_msg";
}

int dat_driveline_msg::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = this->hdr._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->num_motors, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    for (int a0 = 0; a0 < this->num_motors; a0++) {
        tlen = this->motor_data[a0]._encodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fwd_speed, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->yaw_rate, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __boolean_encode_array(buf, offset + pos, maxlen - pos, &this->is_emergency_stop, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __boolean_encode_array(buf, offset + pos, maxlen - pos, &this->is_external_control, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->odometer, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->time_online, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int dat_driveline_msg::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = this->hdr._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->num_motors, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    try {
        this->motor_data.resize(this->num_motors);
    } catch (...) {
        return -1;
    }
    for (int a0 = 0; a0 < this->num_motors; a0++) {
        tlen = this->motor_data[a0]._decodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fwd_speed, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->yaw_rate, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __boolean_decode_array(buf, offset + pos, maxlen - pos, &this->is_emergency_stop, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __boolean_decode_array(buf, offset + pos, maxlen - pos, &this->is_external_control, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->odometer, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->time_online, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int dat_driveline_msg::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += this->hdr._getEncodedSizeNoHash();
    enc_size += __int16_t_encoded_array_size(NULL, 1);
    for (int a0 = 0; a0 < this->num_motors; a0++) {
        enc_size += this->motor_data[a0]._getEncodedSizeNoHash();
    }
    enc_size += __float_encoded_array_size(NULL, 1);
    enc_size += __float_encoded_array_size(NULL, 1);
    enc_size += __boolean_encoded_array_size(NULL, 1);
    enc_size += __boolean_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    return enc_size;
}

uint64_t dat_driveline_msg::_computeHash(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for(fp = p; fp != NULL; fp = fp->parent)
        if(fp->v == dat_driveline_msg::getHash)
            return 0;
    const __lcm_hash_ptr cp = { p, dat_driveline_msg::getHash };

    uint64_t hash = 0xb344a9e2123b0837LL +
         lcmmessages::header::_computeHash(&cp) +
         lcmmessages::MotorTelemetry::_computeHash(&cp);

    return (hash<<1) + ((hash>>63)&1);
}

}

#endif
