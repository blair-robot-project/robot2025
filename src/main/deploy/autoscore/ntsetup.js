// Serializes a value to a MessagePack byte array.
//
// data: The value to serialize. This can be a scalar, array or object.
// options: An object that defined additional options.
// - multiple: Indicates whether multiple values in data are concatenated to multiple MessagePack arrays.
// - invalidTypeReplacement: The value that is used to replace values of unsupported types, or a function that returns such a value, given the original value as parameter.
function serialize(data, options) {
    if (options && options.multiple && !Array.isArray(data)) {
        throw new Error("Invalid argument type: Expected an Array to serialize multiple values.");
    }
    const pow32 = 0x100000000;   // 2^32
    let floatBuffer, floatView;
    let array = new Uint8Array(128);
    let length = 0;

    var th = "";
    if(options && options.typeHint){
        th = options.typeHint;
    }

    if (options && options.multiple) {
        for (let i = 0; i < data.length; i++) {
            append(data[i], false, th);
        }
    }
    else {
        append(data, false, th);
    }
    return array.subarray(0, length);

    function append(data, isReplacement, th) {
        switch (typeof data) {
            case "undefined":
                appendNull(data);
                break;
            case "boolean":
                appendBoolean(data);
                break;
            case "number":
                appendNumber(data, th);
                break;
            case "string":
                appendString(data);
                break;
            case "object":
                if (data === null)
                    appendNull(data);
                else if (data instanceof Date)
                    appendDate(data);
                else if (Array.isArray(data))
                    appendArray(data);
                else if (data instanceof Uint8Array || data instanceof Uint8ClampedArray)
                    appendBinArray(data);
                else if (data instanceof Int8Array || data instanceof Int16Array || data instanceof Uint16Array ||
                    data instanceof Int32Array || data instanceof Uint32Array ||
                    data instanceof Float32Array || data instanceof Float64Array)
                    appendArray(data);
                else
                    appendObject(data);
                break;
            default:
                if (!isReplacement && options && options.invalidTypeReplacement) {
                    if (typeof options.invalidTypeReplacement === "function")
                        append(options.invalidTypeReplacement(data), true, th);
                    else
                        append(options.invalidTypeReplacement, true, th);
                }
                else {
                    throw new Error("Invalid argument type: The type '" + (typeof data) + "' cannot be serialized.");
                }
        }
    }

    function appendNull(data) {
        appendByte(0xc0);
    }

    function appendBoolean(data) {
        appendByte(data ? 0xc3 : 0xc2);
    }

    function appendNumber(data, th) {
        var isInteger = (th === "int") ||
                        (isFinite(data) && Math.floor(data) === data && th !== "double" && th !== "float");
        if (isInteger) {
            // Integer
            if (data >= 0 && data <= 0x7f) {
                appendByte(data);
            }
            else if (data < 0 && data >= -0x20) {
                appendByte(data);
            }
            else if (data > 0 && data <= 0xff) {   // uint8
                appendBytes([0xcc, data]);
            }
            else if (data >= -0x80 && data <= 0x7f) {   // int8
                appendBytes([0xd0, data]);
            }
            else if (data > 0 && data <= 0xffff) {   // uint16
                appendBytes([0xcd, data >>> 8, data]);
            }
            else if (data >= -0x8000 && data <= 0x7fff) {   // int16
                appendBytes([0xd1, data >>> 8, data]);
            }
            else if (data > 0 && data <= 0xffffffff) {   // uint32
                appendBytes([0xce, data >>> 24, data >>> 16, data >>> 8, data]);
            }
            else if (data >= -0x80000000 && data <= 0x7fffffff) {   // int32
                appendBytes([0xd2, data >>> 24, data >>> 16, data >>> 8, data]);
            }
            else if (data > 0 && data <= 0xffffffffffffffff) {   // uint64
                // Split 64 bit number into two 32 bit numbers because JavaScript only regards
                // 32 bits for bitwise operations.
                let hi = data / pow32;
                let lo = data % pow32;
                appendBytes([0xd3, hi >>> 24, hi >>> 16, hi >>> 8, hi, lo >>> 24, lo >>> 16, lo >>> 8, lo]);
            }
            else if (data >= -0x8000000000000000 && data <= 0x7fffffffffffffff) {   // int64
                appendByte(0xd3);
                appendInt64(data);
            }
            else if (data < 0) {   // below int64
                appendBytes([0xd3, 0x80, 0, 0, 0, 0, 0, 0, 0]);
            }
            else {   // above uint64
                appendBytes([0xcf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff]);
            }
        }
        else {
            // Float
            if (!floatView) {
                floatBuffer = new ArrayBuffer(8);
                floatView = new DataView(floatBuffer);
            }
            floatView.setFloat64(0, data);
            appendByte(0xcb);
            appendBytes(new Uint8Array(floatBuffer));
        }
    }

    function appendString(data) {
        let bytes = encodeUtf8(data);
        let length = bytes.length;

        if (length <= 0x1f)
            appendByte(0xa0 + length);
        else if (length <= 0xff)
            appendBytes([0xd9, length]);
        else if (length <= 0xffff)
            appendBytes([0xda, length >>> 8, length]);
        else
            appendBytes([0xdb, length >>> 24, length >>> 16, length >>> 8, length]);

        appendBytes(bytes);
    }

    function appendArray(data) {
        let length = data.length;

        if (length <= 0xf)
            appendByte(0x90 + length);
        else if (length <= 0xffff)
            appendBytes([0xdc, length >>> 8, length]);
        else
            appendBytes([0xdd, length >>> 24, length >>> 16, length >>> 8, length]);

        for (let index = 0; index < length; index++) {
            append(data[index]);
        }
    }

    function appendBinArray(data) {
        let length = data.length;

        if (length <= 0xf)
            appendBytes([0xc4, length]);
        else if (length <= 0xffff)
            appendBytes([0xc5, length >>> 8, length]);
        else
            appendBytes([0xc6, length >>> 24, length >>> 16, length >>> 8, length]);

        appendBytes(data);
    }

    function appendObject(data) {
        let length = 0;
        for (let key in data) {
            if (data[key] !== undefined) {
                length++;
            }
        }

        if (length <= 0xf)
            appendByte(0x80 + length);
        else if (length <= 0xffff)
            appendBytes([0xde, length >>> 8, length]);
        else
            appendBytes([0xdf, length >>> 24, length >>> 16, length >>> 8, length]);

        for (let key in data) {
            let value = data[key];
            if (value !== undefined) {
                append(key);
                append(value);
            }
        }
    }

    function appendDate(data) {
        let sec = data.getTime() / 1000;
        if (data.getMilliseconds() === 0 && sec >= 0 && sec < 0x100000000) {   // 32 bit seconds
            appendBytes([0xd6, 0xff, sec >>> 24, sec >>> 16, sec >>> 8, sec]);
        }
        else if (sec >= 0 && sec < 0x400000000) {   // 30 bit nanoseconds, 34 bit seconds
            let ns = data.getMilliseconds() * 1000000;
            appendBytes([0xd7, 0xff, ns >>> 22, ns >>> 14, ns >>> 6, ((ns << 2) >>> 0) | (sec / pow32), sec >>> 24, sec >>> 16, sec >>> 8, sec]);
        }
        else {   // 32 bit nanoseconds, 64 bit seconds, negative values allowed
            let ns = data.getMilliseconds() * 1000000;
            appendBytes([0xc7, 12, 0xff, ns >>> 24, ns >>> 16, ns >>> 8, ns]);
            appendInt64(sec);
        }
    }

    function appendByte(byte) {
        if (array.length < length + 1) {
            let newLength = array.length * 2;
            while (newLength < length + 1)
                newLength *= 2;
            let newArray = new Uint8Array(newLength);
            newArray.set(array);
            array = newArray;
        }
        array[length] = byte;
        length++;
    }

    function appendBytes(bytes) {
        if (array.length < length + bytes.length) {
            let newLength = array.length * 2;
            while (newLength < length + bytes.length)
                newLength *= 2;
            let newArray = new Uint8Array(newLength);
            newArray.set(array);
            array = newArray;
        }
        array.set(bytes, length);
        length += bytes.length;
    }

    function appendInt64(value) {
        // Split 64 bit number into two 32 bit numbers because JavaScript only regards 32 bits for
        // bitwise operations.
        let hi, lo;
        if (value >= 0) {
            // Same as uint64
            hi = value / pow32;
            lo = value % pow32;
        }
        else {
            // Split absolute value to high and low, then NOT and ADD(1) to restore negativity
            value++;
            hi = Math.abs(value) / pow32;
            lo = Math.abs(value) % pow32;
            hi = ~hi;
            lo = ~lo;
        }
        appendBytes([hi >>> 24, hi >>> 16, hi >>> 8, hi, lo >>> 24, lo >>> 16, lo >>> 8, lo]);
    }
}

// Deserializes a MessagePack byte array to a value.
//
// array: The MessagePack byte array to deserialize. This must be an Array or Uint8Array containing bytes, not a string.
// options: An object that defined additional options.
// - multiple: Indicates whether multiple concatenated MessagePack arrays are returned as an array.
function deserialize(array, options) {
    const pow32 = 0x100000000;   // 2^32
    let pos = 0;
    if (array instanceof ArrayBuffer) {
        array = new Uint8Array(array);
    }
    if (typeof array !== "object" || typeof array.length === "undefined") {
        throw new Error("Invalid argument type: Expected a byte array (Array or Uint8Array) to deserialize.");
    }
    if (!array.length) {
        throw new Error("Invalid argument: The byte array to deserialize is empty.");
    }
    if (!(array instanceof Uint8Array)) {
        array = new Uint8Array(array);
    }
    let data;
    if (options && options.multiple) {
        // Read as many messages as are available
        data = [];
        while (pos < array.length) {
            data.push(read());
        }
    }
    else {
        // Read only one message and ignore additional data
        data = read();
    }
    return data;

    function read() {
        const byte = array[pos++];
        if (byte >= 0x00 && byte <= 0x7f) return byte;   // positive fixint
        if (byte >= 0x80 && byte <= 0x8f) return readMap(byte - 0x80);   // fixmap
        if (byte >= 0x90 && byte <= 0x9f) return readArray(byte - 0x90);   // fixarray
        if (byte >= 0xa0 && byte <= 0xbf) return readStr(byte - 0xa0);   // fixstr
        if (byte === 0xc0) return null;   // nil
        if (byte === 0xc1) throw new Error("Invalid byte code 0xc1 found.");   // never used
        if (byte === 0xc2) return false;   // false
        if (byte === 0xc3) return true;   // true
        if (byte === 0xc4) return readBin(-1, 1);   // bin 8
        if (byte === 0xc5) return readBin(-1, 2);   // bin 16
        if (byte === 0xc6) return readBin(-1, 4);   // bin 32
        if (byte === 0xc7) return readExt(-1, 1);   // ext 8
        if (byte === 0xc8) return readExt(-1, 2);   // ext 16
        if (byte === 0xc9) return readExt(-1, 4);   // ext 32
        if (byte === 0xca) return readFloat(4);   // float 32
        if (byte === 0xcb) return readFloat(8);   // float 64
        if (byte === 0xcc) return readUInt(1);   // uint 8
        if (byte === 0xcd) return readUInt(2);   // uint 16
        if (byte === 0xce) return readUInt(4);   // uint 32
        if (byte === 0xcf) return readUInt(8);   // uint 64
        if (byte === 0xd0) return readInt(1);   // int 8
        if (byte === 0xd1) return readInt(2);   // int 16
        if (byte === 0xd2) return readInt(4);   // int 32
        if (byte === 0xd3) return readInt(8);   // int 64
        if (byte === 0xd4) return readExt(1);   // fixext 1
        if (byte === 0xd5) return readExt(2);   // fixext 2
        if (byte === 0xd6) return readExt(4);   // fixext 4
        if (byte === 0xd7) return readExt(8);   // fixext 8
        if (byte === 0xd8) return readExt(16);   // fixext 16
        if (byte === 0xd9) return readStr(-1, 1);   // str 8
        if (byte === 0xda) return readStr(-1, 2);   // str 16
        if (byte === 0xdb) return readStr(-1, 4);   // str 32
        if (byte === 0xdc) return readArray(-1, 2);   // array 16
        if (byte === 0xdd) return readArray(-1, 4);   // array 32
        if (byte === 0xde) return readMap(-1, 2);   // map 16
        if (byte === 0xdf) return readMap(-1, 4);   // map 32
        if (byte >= 0xe0 && byte <= 0xff) return byte - 256;   // negative fixint
        console.debug("msgpack array:", array);
        throw new Error("Invalid byte value '" + byte + "' at index " + (pos - 1) + " in the MessagePack binary data (length " + array.length + "): Expecting a range of 0 to 255. This is not a byte array.");
    }

    function readInt(size) {
        let value = 0;
        let first = true;
        while (size-- > 0) {
            if (first) {
                let byte = array[pos++];
                value += byte & 0x7f;
                if (byte & 0x80) {
                    value -= 0x80;   // Treat most-significant bit as -2^i instead of 2^i
                }
                first = false;
            }
            else {
                value *= 256;
                value += array[pos++];
            }
        }
        return value;
    }

    function readUInt(size) {
        let value = 0;
        while (size-- > 0) {
            value *= 256;
            value += array[pos++];
        }
        return value;
    }

    function readFloat(size) {
        let view = new DataView(array.buffer, pos + array.byteOffset, size);
        pos += size;
        if (size === 4)
            return view.getFloat32(0, false);
        if (size === 8)
            return view.getFloat64(0, false);
    }

    function readBin(size, lengthSize) {
        if (size < 0) size = readUInt(lengthSize);
        let data = array.subarray(pos, pos + size);
        pos += size;
        return data;
    }

    function readMap(size, lengthSize) {
        if (size < 0) size = readUInt(lengthSize);
        let data = {};
        while (size-- > 0) {
            let key = read();
            data[key] = read();
        }
        return data;
    }

    function readArray(size, lengthSize) {
        if (size < 0) size = readUInt(lengthSize);
        let data = [];
        while (size-- > 0) {
            data.push(read());
        }
        return data;
    }

    function readStr(size, lengthSize) {
        if (size < 0) size = readUInt(lengthSize);
        let start = pos;
        pos += size;
        return decodeUtf8(array, start, size);
    }

    function readExt(size, lengthSize) {
        if (size < 0) size = readUInt(lengthSize);
        let type = readUInt(1);
        let data = readBin(size);
        switch (type) {
            case 255:
                return readExtDate(data);
        }
        return { type: type, data: data };
    }

    function readExtDate(data) {
        if (data.length === 4) {
            let sec = ((data[0] << 24) >>> 0) +
                ((data[1] << 16) >>> 0) +
                ((data[2] << 8) >>> 0) +
                data[3];
            return new Date(sec * 1000);
        }
        if (data.length === 8) {
            let ns = ((data[0] << 22) >>> 0) +
                ((data[1] << 14) >>> 0) +
                ((data[2] << 6) >>> 0) +
                (data[3] >>> 2);
            let sec = ((data[3] & 0x3) * pow32) +
                ((data[4] << 24) >>> 0) +
                ((data[5] << 16) >>> 0) +
                ((data[6] << 8) >>> 0) +
                data[7];
            return new Date(sec * 1000 + ns / 1000000);
        }
        if (data.length === 12) {
            let ns = ((data[0] << 24) >>> 0) +
                ((data[1] << 16) >>> 0) +
                ((data[2] << 8) >>> 0) +
                data[3];
            pos -= 8;
            let sec = readInt(8);
            return new Date(sec * 1000 + ns / 1000000);
        }
        throw new Error("Invalid data length for a date value.");
    }
}

// Encodes a string to UTF-8 bytes.
function encodeUtf8(str) {
    // Prevent excessive array allocation and slicing for all 7-bit characters
    let ascii = true, length = str.length;
    for (let x = 0; x < length; x++) {
        if (str.charCodeAt(x) > 127) {
            ascii = false;
            break;
        }
    }

    // Based on: https://gist.github.com/pascaldekloe/62546103a1576803dade9269ccf76330
    let i = 0, bytes = new Uint8Array(str.length * (ascii ? 1 : 4));
    for (let ci = 0; ci !== length; ci++) {
        let c = str.charCodeAt(ci);
        if (c < 128) {
            bytes[i++] = c;
            continue;
        }
        if (c < 2048) {
            bytes[i++] = c >> 6 | 192;
        }
        else {
            if (c > 0xd7ff && c < 0xdc00) {
                if (++ci >= length)
                    throw new Error("UTF-8 encode: incomplete surrogate pair");
                let c2 = str.charCodeAt(ci);
                if (c2 < 0xdc00 || c2 > 0xdfff)
                    throw new Error("UTF-8 encode: second surrogate character 0x" + c2.toString(16) + " at index " + ci + " out of range");
                c = 0x10000 + ((c & 0x03ff) << 10) + (c2 & 0x03ff);
                bytes[i++] = c >> 18 | 240;
                bytes[i++] = c >> 12 & 63 | 128;
            }
            else bytes[i++] = c >> 12 | 224;
            bytes[i++] = c >> 6 & 63 | 128;
        }
        bytes[i++] = c & 63 | 128;
    }
    return ascii ? bytes : bytes.subarray(0, i);
}

// Decodes a string from UTF-8 bytes.
function decodeUtf8(bytes, start, length) {
    // Based on: https://gist.github.com/pascaldekloe/62546103a1576803dade9269ccf76330
    let i = start, str = "";
    length += start;
    while (i < length) {
        let c = bytes[i++];
        if (c > 127) {
            if (c > 191 && c < 224) {
                if (i >= length)
                    throw new Error("UTF-8 decode: incomplete 2-byte sequence");
                c = (c & 31) << 6 | bytes[i++] & 63;
            }
            else if (c > 223 && c < 240) {
                if (i + 1 >= length)
                    throw new Error("UTF-8 decode: incomplete 3-byte sequence");
                c = (c & 15) << 12 | (bytes[i++] & 63) << 6 | bytes[i++] & 63;
            }
            else if (c > 239 && c < 248) {
                if (i + 2 >= length)
                    throw new Error("UTF-8 decode: incomplete 4-byte sequence");
                c = (c & 7) << 18 | (bytes[i++] & 63) << 12 | (bytes[i++] & 63) << 6 | bytes[i++] & 63;
            }
            else throw new Error("UTF-8 decode: unknown multibyte start 0x" + c.toString(16) + " at index " + (i - 1));
        }
        if (c <= 0xffff) str += String.fromCharCode(c);
        else if (c <= 0x10ffff) {
            c -= 0x10000;
            str += String.fromCharCode(c >> 10 | 0xd800)
            str += String.fromCharCode(c & 0x3FF | 0xdc00)
        }
        else throw new Error("UTF-8 decode: code point 0x" + c.toString(16) + " exceeds UTF-16 reach");
    }
    return str;
}

// The exported functions
let msgpack = {
    serialize: serialize,
    deserialize: deserialize,

    // Compatibility with other libraries
    encode: serialize,
    decode: deserialize
};

// Environment detection
if (typeof module === "object" && module && typeof module.exports === "object") {
    // Node.js
    module.exports = msgpack;
}
else {
    // Global object
    window[window.msgpackJsName || "msgpack"] = msgpack;
}

/**
 * nt4.js - Pure-javascript module implementation of the NetworkTables 4 spec 
 *          for the FIRST robotics Competition
 * 
 * See https://github.com/wpilibsuite/allwpilib/blob/main/ntcore/doc/networktables4.adoc
 * for the full spec.
 */


/**
 * Lookup from type string to type integer
 */
var typestrIdxLookup = {
    NT4_TYPESTR: 0,
    "double": 1,
    "int": 2,
    "float": 3,
    "string": 4,
    "json": 4,
    "raw": 5,
    "rpc": 5,
    "msgpack": 5,
    "protobuf": 5,
    "boolean[]": 16,
    "double[]": 17,
    "int[]": 18,
    "float[]": 19,
    "string[]": 20
}


/**
 * JS Definition of the topic type strings
 */
class NT4_TYPESTR {
    static BOOL = "boolean";
    static FLOAT_64 = "double";
    static INT = "int";
    static FLOAT_32 = "float";
    static STR = "string";
    static JSON = "json";
    static BIN_RAW = "raw";
    static BIN_RPC = "rpc";
    static BIN_MSGPACK = "msgpack";
    static BIN_PROTOBUF = "protobuf";
    static BOOL_ARR = "boolean[]";
    static FLOAT_64_ARR = "double[]";
    static INT_ARR = "int[]";
    static FLOAT_32_ARR = "float[]";
    static STR_ARR = "string[]";
}

/**
 * Class to describe a client's subscription to topics
 */
class NT4_Subscription {
    topics = new Set();
    options = new NT4_SubscriptionOptions();
    uid = -1;

    toSubscribeObj() {
        return {
            "topics": Array.from(this.topics),
            "options": this.options.toObj(),
            "subuid": this.uid,
        };
    }

    toUnSubscribeObj() {
        return {
            "subuid": this.uid,
        };
    }
}

/**
 * Class to describe the options associated with a client's subscription to topics
 */
class NT4_SubscriptionOptions {
    periodicRate_s = 0.1;
    all = false;
    topicsonly = false;
    prefix = true; //nonstandard default

    toObj() {
        return {
            "periodic": this.periodicRate_s,
            "all": this.all,
            "topicsonly": this.topicsonly,
            "prefix": this.prefix,
        };
    }
}

/**
 * Class to describe a topic that the client and server both know about
 */
class NT4_Topic {
    name = "";
    type = "";
    id = 0;
    pubuid = 0;
    properties = {}; //Properties are free-form, might have anything in them

    toPublishObj() {
        return {
            "name": this.name,
            "type": this.type,
            "pubuid": this.pubuid,
        }
    }

    toUnPublishObj() {
        return {
            "name": this.name,
            "pubuid": this.pubuid,
        }
    }

    toPropertiesObj() {
        return {
            "name": this.name,
            "update": this.properties,
        }
    }

    getTypeIdx() {
        return typestrIdxLookup[this.type];
    }

    getPropertiesString(){
        var retStr = "{"
        for (var key in this.properties){
            retStr += key + ":" + this.properties[key] + ", ";
        }
        if(Object.keys(this.properties).length == 1) {
            retStr = retStr.slice(0, -2);
        }
        retStr += "}";
        return retStr;
    }
}

class NT4_Client {


    /**
     * Main client class. User code should instantiate one of these.
     * Client will immediately start to try to connect to the server on instantiation
     * and continue to reconnect in the background if disconnected.
     * As long as the server is connected, time synchronization will occur in the background.
     * @param {string} serverAddr String representing the network address of the server
     * @param {function} onTopicAnnounce_in User-supplied callback function for whenever a new topic is announced.
     * @param {function} onTopicUnAnnounce_in User-supplied callback function for whenever a topic is unnanounced.
     * @param {function} onNewTopicData_in User-supplied callback function for when the client gets new values for a topic
     * @param {function} onParameterChange User-supplied callback function for when a parameter changes
     * @param {function} onConnect_in User-supplied callback function for when the client successfully connects to the server
     * @param {function} onDisconnect_in User-supplied callback for when the client is disconnected from the server
     */
    constructor(serverAddr,
        onTopicAnnounce_in,   
        onTopicUnAnnounce_in, 
        onNewTopicData_in,    
        onParameterChange,
        onConnect_in,         
        onDisconnect_in) {    

        this.onTopicAnnounce = onTopicAnnounce_in;
        this.onTopicUnAnnounce = onTopicUnAnnounce_in;
        this.onNewTopicData = onNewTopicData_in;
        this.onConnect = onConnect_in;
        this.paramChange = onParameterChange;
        this.onDisconnect = onDisconnect_in;

        this.subscriptions = new Map();
        this.subscription_uid_counter = 0;
        this.publish_uid_counter = 0;

        this.clientPublishedTopics = new Map();
        this.announcedTopics = new Map();

        this.timeSyncBgEvent = setInterval(this.ws_sendTimestamp.bind(this), 5000);

        // WS Connection State (with defaults)
        this.serverBaseAddr = serverAddr;
        this.clientIdx = 0;
        this.serverAddr = "";
        this.serverConnectionActive = false;
        this.serverTimeOffset_us = 0;

        //Trigger the websocket to connect automatically
        this.ws_connect();

    }

    //////////////////////////////////////////////////////////////
    // PUBLIC API

    /**
     * Add a new subscription which requests announcement of topics, but no data
     * Generally, this must be called first before the server will announce any topics.
     * @param {List<String>} topicPatterns wildcard-enabled list of patterns that this client will care about.
     * @returns a NT4_Subscription object describing the subscription
     */
    subscribeTopicNames(topicPatterns) {
        var newSub = new NT4_Subscription();
        newSub.uid = this.getNewSubUID();
        newSub.options.topicsonly = true;
        newSub.options.periodicRate_s = 1.0;
        newSub.topics = new Set(topicPatterns);

        this.subscriptions.set(newSub.uid, newSub);
        if (this.serverConnectionActive) {
            this.ws_subscribe(newSub);
        }
        return newSub;
    }

    /**
     * Subscribe to topics, requesting the server send value updates periodically.
     * This means the server may skip sending some value updates.
     * @param {List<String>} topicPatterns wildcard-enabled list of patterns that this client wants data from.
     * @param {double} period Requested data rate, in seconds
     * @returns a NT4_Subscription object describing the subscription
     */
    subscribePeriodic(topicPatterns, period) {
        var newSub = new NT4_Subscription();
        newSub.uid = this.getNewSubUID();
        newSub.options.periodicRate_s = period;
        newSub.topics = new Set(topicPatterns);

        this.subscriptions.set(newSub.uid, newSub);
        if (this.serverConnectionActive) {
            this.ws_subscribe(newSub);
        }
        return newSub;
    }

    /**
     * Subscribe to topics, requesting the server send all value updates. 
     * @param {List<String>} topicPatterns wildcard-enabled list of patterns that this client wants data from.
     * @returns a NT4_Subscription object describing the subscription
     */
    subscribeAllSamples(topicPatterns) {
        var newSub = new NT4_Subscription();
        newSub.uid = this.getNewSubUID();
        newSub.topics = new Set(topicPatterns);
        newSub.options.all = true;

        this.subscriptions.set(newSub.uid, newSub);
        if (this.serverConnectionActive) {
            this.ws_subscribe(newSub);
        }
        return newSub;
    }

    /**
     * Request the server stop sending value updates and topic announcements
     * @param {NT4_Subscription} sub The subscription object generated by a call to a subscribe*() method.
     */
    unSubscribe(sub) {
        this.subscriptions.delete(sub.uid);
        if (this.serverConnectionActive) {
            this.ws_unsubscribe(sub);
        }
    }

    /**
     * Unsubscribe from all current subscriptions
     */
    clearAllSubscriptions() {
        for (const sub of this.subscriptions.values()) {
            this.unSubscribe(sub);
        }
    }

    /**
     * Set the properties of a particular topic
     * @param {NT4_Topic} topic the topic to update
     * @param {boolean} isPersistent set whether the topic should be persistent
     * @param {boolean} isRetained set whether the topic should be retained
     */
    setProperties(topic, isPersistent, isRetained) {
        topic.properties.persistent = isPersistent;
        topic.properties.retained = isRetained;
        if (this.serverConnectionActive) {
            this.ws_setproperties(topic);
        }
    }

    /**
     * Publish a new topic from this client
     * @param {String} name Topic's full name
     * @param {NT4_TYPESTR} type Topic Type
     * @returns 
     */
    publishNewTopic(name, type) {
        var newTopic = new NT4_Topic();
        newTopic.name = name;
        newTopic.type = type;
        this.publishTopic(newTopic);
        return newTopic;
    }

    /**
     * Publish a new topic from this client
     * @param {NT4_Topic} topic the topic to publish
     * @returns 
     */
    publishTopic(topic) {
        topic.pubuid = this.getNewPubUID();
        this.clientPublishedTopics.set(topic.name, topic);
        if (this.serverConnectionActive) {
            this.ws_publish(topic);
        }
    }

    /**
     * Un-Publish a previously-published topic from this client
     * @param {NT4_Topic} oldTopic the topic to un-publish
     * @returns 
     */    
    unPublishTopic(oldTopic) {
        this.clientPublishedTopics.delete(oldTopic.name);
        if (this.serverConnectionActive) {
            this.ws_unpublish(oldTopic);
        }
    }

    /**
     * Send some new value to the server
     * Timestamp is whatever the current time is.
     * @param {NT4_Topic} topic The topic to update a value for
     * @param {*} value The value to pass in
     */
    addSample(topic, value) {
        var timestamp = this.getServerTime_us();
        this.addSample(topic, timestamp, value);
    }

    /**
     * Send some new value to the server
     * Timestamp is whatever the current time is.
     * @param {NT4_Topic} topic The topic to update a value for
     * @param {double} timestamp The server time at which the update happened, in microseconds. Should be a value returned from getServerTime_us().
     * @param {*} value The value to pass in
     */
    addSample(topic, timestamp, value) {
        if (typeof topic === 'string') {
            var topicFound = false;
            //Slow-lookup - strings are assumed to be topic names for things the server has already announced.
            for (const topicIter of this.announcedTopics.values()) {
                if (topicIter.name === topic) {
                    topic = topicIter;
                    topicFound = true;
                    break;
                }
            }
            if (!topicFound) {
                throw "Topic " + topic + " not found in announced server topics!";
            }
        }

        var sourceData = [topic.pubuid, timestamp, topic.getTypeIdx(), value];
        var txData = msgpack.serialize(sourceData);
        if(topic.name != "Time") {
            console.log("\n\n\n");
            console.log(value);
            console.log(topic);
            console.log("\n\n\n");
        }


        this.ws_sendBinary(txData);
    }

    /**
     * Gets the server time. This is equal to the client current time, offset
     * by the most recent results from running Cristian’s Algorithm with the server
     * to synchronize timebases. 
     * @returns The current time on the server, in microseconds
     */
    getServerTime_us() {
        return this.getClientTime_us() + this.serverTimeOffset_us;
    }

    //////////////////////////////////////////////////////////////
    // Server/Client Time Sync Handling

    getClientTime_us() {
        return Math.round(performance.now() * 1000.0);
    }

    ws_sendTimestamp() {
        var timeTopic = this.announcedTopics.get(-1);
        if (timeTopic) {
            var timeToSend = this.getClientTime_us();
            this.addSample(timeTopic, 0, timeToSend);
        }
    }

    ws_handleReceiveTimestamp(serverTimestamp, clientTimestamp) {
        var rxTime = this.getClientTime_us();

        //Recalculate server/client offset based on round trip time
        var rtt = rxTime - clientTimestamp;
        var serverTimeAtRx = serverTimestamp - rtt / 2.0;
        this.serverTimeOffset_us = serverTimeAtRx - rxTime;

    }

    //////////////////////////////////////////////////////////////
    // Websocket Message Send Handlers

    ws_subscribe(sub) {
        this.ws_sendJSON("subscribe", sub.toSubscribeObj());
    }

    ws_unsubscribe(sub) {
        this.ws_sendJSON("unsubscribe", sub.toUnSubscribeObj());
    }

    ws_publish(topic) {
        this.ws_sendJSON("publish", topic.toPublishObj());
    }

    ws_unpublish(topic) {
        this.ws_sendJSON("unpublish", topic.toUnPublishObj());
    }

    ws_setproperties(topic) {
        this.ws_sendJSON("setproperties", topic.toPropertiesObj());
    }

    ws_sendJSON(method, params) { //Sends a single json message
        if (this.ws.readyState === WebSocket.OPEN) {
            var txObj = [{
                "method": method,
                "params": params
            }];
            var txJSON = JSON.stringify(txObj);

            console.log("[NT4] Client Says: " + txJSON);

            this.ws.send(txJSON);
        }
    }

    ws_sendBinary(data) {
        if (this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(data);
        }
    }

    //////////////////////////////////////////////////////////////
    // Websocket connection Maintenance

    ws_onOpen() {

        // Add default time topic
        var timeTopic = new NT4_Topic();
        timeTopic.name = "Time";
        timeTopic.id = -1;
        timeTopic.pubuid = -1;
        timeTopic.type = NT4_TYPESTR.INT;
        this.announcedTopics.set(timeTopic.id, timeTopic);

        // Set the flag allowing general server communication
        this.serverConnectionActive = true;

        //Publish any existing topics
        for (const topic of this.clientPublishedTopics.values()) {
            this.ws_publish(topic);
            this.ws_setproperties(topic);
        }

        //Subscribe to existing subscriptions
        for (const sub of this.subscriptions.values()) {
            this.ws_subscribe(sub);
        }

        // User connection-opened hook
        this.onConnect();
    }

    ws_onClose(e) {
        //Clear flags to stop server communication
        this.ws = null;
        this.serverConnectionActive = false;

        // User connection-closed hook
        this.onDisconnect();

        //Clear out any local cache of server state
        this.announcedTopics.clear();
        console.log('[NT4] Socket is closed. Reconnect will be attempted in 0.5 seconds.', e.reason);
        setTimeout(this.ws_connect.bind(this), 500);

        if (!e.wasClean) {
            console.error('Socket encountered error!');
        }

    }

    ws_onError(e) {
        console.log("[NT4] Websocket error - " + e.toString());
        this.ws.close();
    }

    ws_onMessage(e) {
        if (typeof e.data === 'string') {
            console.log("1064")
            console.log("[NT4] Server Says: " + e.data);
            //JSON Message
            var rxArray = JSON.parse(e.data);

            rxArray.forEach(function (msg) {

                //Validate proper format of message
                if (typeof msg !== 'object') {
                    console.log("[NT4] Ignoring text message, JSON parsing did not produce an object.");
                    return;
                }

                if (!("method" in msg) || !("params" in msg)) {
                    console.log("[NT4] Ignoring text message, JSON parsing did not find all required fields.");
                    return;
                }

                var method = msg["method"];
                var params = msg["params"];

                if (typeof method !== 'string') {
                    console.log("[NT4] Ignoring text message, JSON parsing found \"method\", but it wasn't a string.");
                    return;
                }

                if (typeof params !== 'object') {
                    console.log("[NT4] Ignoring text message, JSON parsing found \"params\", but it wasn't an object.");
                    return;
                }

                // Message validates reasonably, switch based on supported methods
                console.log(method)
                if (method === "announce") {

                    //Check to see if we already knew about this topic. If not, make a new object.

                    var newTopic = null;
                    for (const topic of this.clientPublishedTopics.values()) {
                        if (params.name === topic.name) {
                            newTopic = topic; //Existing topic, use it.
                        }
                    }

                    // Did not know about the topic. Make a new one.
                    if(newTopic === null){
                        newTopic = new NT4_Topic();
                    }

                    newTopic.name = params.name;
                    newTopic.id = params.id;

                    //Strategy - if server sends a pubid use it
                    // otherwise, preserve whatever we had?
                    //TODO - ask peter about this. It smells wrong.
                    if (params.pubid != null) {
                        newTopic.pubuid = params.pubuid;
                    }

                    newTopic.type = params.type;
                    newTopic.properties = params.properties;
                    this.announcedTopics.set(newTopic.id, newTopic);
                    this.onTopicAnnounce(newTopic);
                } else if (method === "unannounce") {
                    var removedTopic = this.announcedTopics.get(params.id);
                    if (!removedTopic) {
                        console.log("[NT4] Ignorining unannounce, topic was not previously announced.");
                        return;
                    }
                    this.announcedTopics.delete(removedTopic.id);
                    this.onTopicUnAnnounce(removedTopic);

                } else if (method === "properties") {
                    //TODO support property changes
                    //fine, i'll do it myself - edem
                    console.log("[NT4] Ignoring properties, not implemented yet.");
                    console.log(params)
                    let change = params.update
                    this.paramChange(change);
                } else {
                    console.log("[NT4] Ignoring text message - unknown method " + method);
                    return;
                }
            }, this);

        } else {
            //MSGPack
            var rxArray = msgpack.deserialize(e.data, { multiple: true });

            rxArray.forEach(function (unpackedData) { //For every value update...
                var topicID = unpackedData[0];
                var timestamp_us = unpackedData[1];
                var typeIdx = unpackedData[2];
                var value = unpackedData[3];

                if (topicID >= 0) {
                    var topic = this.announcedTopics.get(topicID);
                    this.onNewTopicData(topic, timestamp_us, value);
                } else if (topicID === -1) {
                    this.ws_handleReceiveTimestamp(timestamp_us, value);
                } else {
                    console.log("[NT4] Ignoring binary data - invalid topic id " + topicID.toString());
                }
            }, this);

        }
    }

    ws_connect() {

        this.clientIdx = 449;

        var port = 5810;
        var prefix = "ws://";

        this.serverAddr = prefix + "10.4.49.2" + ":" + port.toString() + "/nt/" + "JSClient_" + this.clientIdx.toString();

        this.ws = new WebSocket(this.serverAddr, "networktables.first.wpi.edu");
        this.ws.binaryType = "arraybuffer";
        this.ws.onopen = this.ws_onOpen.bind(this);
        this.ws.onmessage = this.ws_onMessage.bind(this);
        this.ws.onclose = this.ws_onClose.bind(this);
        this.ws.onerror = this.ws_onError.bind(this);

        console.log("[NT4] Connected with idx " + this.clientIdx.toString());
    }



    //////////////////////////////////////////////////////////////
    // General utilties

    getNewSubUID() {
        this.subscription_uid_counter++;
        return this.subscription_uid_counter + this.clientIdx;
    }

    getNewPubUID() {
        this.publish_uid_counter++;
        return this.publish_uid_counter + this.clientIdx;
    }


}