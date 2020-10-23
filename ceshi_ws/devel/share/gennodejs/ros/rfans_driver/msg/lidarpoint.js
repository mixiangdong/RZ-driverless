// Auto-generated. Do not edit!

// (in-package rfans_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class lidarpoint {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.A_X = null;
      this.A_Y = null;
      this.B_X = null;
      this.B_Y = null;
      this.a_x = null;
      this.a_y = null;
      this.b_x = null;
      this.b_y = null;
      this.Aa_Xx = null;
      this.Aa_Yy = null;
      this.Bb_Xx = null;
      this.Bb_Yy = null;
    }
    else {
      if (initObj.hasOwnProperty('A_X')) {
        this.A_X = initObj.A_X
      }
      else {
        this.A_X = 0.0;
      }
      if (initObj.hasOwnProperty('A_Y')) {
        this.A_Y = initObj.A_Y
      }
      else {
        this.A_Y = 0.0;
      }
      if (initObj.hasOwnProperty('B_X')) {
        this.B_X = initObj.B_X
      }
      else {
        this.B_X = 0.0;
      }
      if (initObj.hasOwnProperty('B_Y')) {
        this.B_Y = initObj.B_Y
      }
      else {
        this.B_Y = 0.0;
      }
      if (initObj.hasOwnProperty('a_x')) {
        this.a_x = initObj.a_x
      }
      else {
        this.a_x = 0.0;
      }
      if (initObj.hasOwnProperty('a_y')) {
        this.a_y = initObj.a_y
      }
      else {
        this.a_y = 0.0;
      }
      if (initObj.hasOwnProperty('b_x')) {
        this.b_x = initObj.b_x
      }
      else {
        this.b_x = 0.0;
      }
      if (initObj.hasOwnProperty('b_y')) {
        this.b_y = initObj.b_y
      }
      else {
        this.b_y = 0.0;
      }
      if (initObj.hasOwnProperty('Aa_Xx')) {
        this.Aa_Xx = initObj.Aa_Xx
      }
      else {
        this.Aa_Xx = 0.0;
      }
      if (initObj.hasOwnProperty('Aa_Yy')) {
        this.Aa_Yy = initObj.Aa_Yy
      }
      else {
        this.Aa_Yy = 0.0;
      }
      if (initObj.hasOwnProperty('Bb_Xx')) {
        this.Bb_Xx = initObj.Bb_Xx
      }
      else {
        this.Bb_Xx = 0.0;
      }
      if (initObj.hasOwnProperty('Bb_Yy')) {
        this.Bb_Yy = initObj.Bb_Yy
      }
      else {
        this.Bb_Yy = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type lidarpoint
    // Serialize message field [A_X]
    bufferOffset = _serializer.float64(obj.A_X, buffer, bufferOffset);
    // Serialize message field [A_Y]
    bufferOffset = _serializer.float64(obj.A_Y, buffer, bufferOffset);
    // Serialize message field [B_X]
    bufferOffset = _serializer.float64(obj.B_X, buffer, bufferOffset);
    // Serialize message field [B_Y]
    bufferOffset = _serializer.float64(obj.B_Y, buffer, bufferOffset);
    // Serialize message field [a_x]
    bufferOffset = _serializer.float64(obj.a_x, buffer, bufferOffset);
    // Serialize message field [a_y]
    bufferOffset = _serializer.float64(obj.a_y, buffer, bufferOffset);
    // Serialize message field [b_x]
    bufferOffset = _serializer.float64(obj.b_x, buffer, bufferOffset);
    // Serialize message field [b_y]
    bufferOffset = _serializer.float64(obj.b_y, buffer, bufferOffset);
    // Serialize message field [Aa_Xx]
    bufferOffset = _serializer.float64(obj.Aa_Xx, buffer, bufferOffset);
    // Serialize message field [Aa_Yy]
    bufferOffset = _serializer.float64(obj.Aa_Yy, buffer, bufferOffset);
    // Serialize message field [Bb_Xx]
    bufferOffset = _serializer.float64(obj.Bb_Xx, buffer, bufferOffset);
    // Serialize message field [Bb_Yy]
    bufferOffset = _serializer.float64(obj.Bb_Yy, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type lidarpoint
    let len;
    let data = new lidarpoint(null);
    // Deserialize message field [A_X]
    data.A_X = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [A_Y]
    data.A_Y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [B_X]
    data.B_X = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [B_Y]
    data.B_Y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [a_x]
    data.a_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [a_y]
    data.a_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [b_x]
    data.b_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [b_y]
    data.b_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Aa_Xx]
    data.Aa_Xx = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Aa_Yy]
    data.Aa_Yy = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Bb_Xx]
    data.Bb_Xx = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Bb_Yy]
    data.Bb_Yy = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 96;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rfans_driver/lidarpoint';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a84347ea08c5142003aeb6f79e914d56';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #大写表示左侧红锥桶，小写表示右侧蓝色锥桶;双写字母例如Aa表示黄色起始锥桶
    float64 A_X
    float64 A_Y
    float64 B_X
    float64 B_Y
    float64 a_x
    float64 a_y
    float64 b_x
    float64 b_y
    float64 Aa_Xx
    float64 Aa_Yy
    float64 Bb_Xx
    float64 Bb_Yy
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new lidarpoint(null);
    if (msg.A_X !== undefined) {
      resolved.A_X = msg.A_X;
    }
    else {
      resolved.A_X = 0.0
    }

    if (msg.A_Y !== undefined) {
      resolved.A_Y = msg.A_Y;
    }
    else {
      resolved.A_Y = 0.0
    }

    if (msg.B_X !== undefined) {
      resolved.B_X = msg.B_X;
    }
    else {
      resolved.B_X = 0.0
    }

    if (msg.B_Y !== undefined) {
      resolved.B_Y = msg.B_Y;
    }
    else {
      resolved.B_Y = 0.0
    }

    if (msg.a_x !== undefined) {
      resolved.a_x = msg.a_x;
    }
    else {
      resolved.a_x = 0.0
    }

    if (msg.a_y !== undefined) {
      resolved.a_y = msg.a_y;
    }
    else {
      resolved.a_y = 0.0
    }

    if (msg.b_x !== undefined) {
      resolved.b_x = msg.b_x;
    }
    else {
      resolved.b_x = 0.0
    }

    if (msg.b_y !== undefined) {
      resolved.b_y = msg.b_y;
    }
    else {
      resolved.b_y = 0.0
    }

    if (msg.Aa_Xx !== undefined) {
      resolved.Aa_Xx = msg.Aa_Xx;
    }
    else {
      resolved.Aa_Xx = 0.0
    }

    if (msg.Aa_Yy !== undefined) {
      resolved.Aa_Yy = msg.Aa_Yy;
    }
    else {
      resolved.Aa_Yy = 0.0
    }

    if (msg.Bb_Xx !== undefined) {
      resolved.Bb_Xx = msg.Bb_Xx;
    }
    else {
      resolved.Bb_Xx = 0.0
    }

    if (msg.Bb_Yy !== undefined) {
      resolved.Bb_Yy = msg.Bb_Yy;
    }
    else {
      resolved.Bb_Yy = 0.0
    }

    return resolved;
    }
};

module.exports = lidarpoint;
