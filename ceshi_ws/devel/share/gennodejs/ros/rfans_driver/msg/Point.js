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

class Point {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x1 = null;
      this.y1 = null;
      this.x2 = null;
      this.y2 = null;
      this.a1 = null;
      this.a2 = null;
      this.a3 = null;
      this.a4 = null;
      this.b1 = null;
      this.b2 = null;
      this.b3 = null;
      this.b4 = null;
    }
    else {
      if (initObj.hasOwnProperty('x1')) {
        this.x1 = initObj.x1
      }
      else {
        this.x1 = 0.0;
      }
      if (initObj.hasOwnProperty('y1')) {
        this.y1 = initObj.y1
      }
      else {
        this.y1 = 0.0;
      }
      if (initObj.hasOwnProperty('x2')) {
        this.x2 = initObj.x2
      }
      else {
        this.x2 = 0.0;
      }
      if (initObj.hasOwnProperty('y2')) {
        this.y2 = initObj.y2
      }
      else {
        this.y2 = 0.0;
      }
      if (initObj.hasOwnProperty('a1')) {
        this.a1 = initObj.a1
      }
      else {
        this.a1 = 0.0;
      }
      if (initObj.hasOwnProperty('a2')) {
        this.a2 = initObj.a2
      }
      else {
        this.a2 = 0.0;
      }
      if (initObj.hasOwnProperty('a3')) {
        this.a3 = initObj.a3
      }
      else {
        this.a3 = 0.0;
      }
      if (initObj.hasOwnProperty('a4')) {
        this.a4 = initObj.a4
      }
      else {
        this.a4 = 0.0;
      }
      if (initObj.hasOwnProperty('b1')) {
        this.b1 = initObj.b1
      }
      else {
        this.b1 = 0.0;
      }
      if (initObj.hasOwnProperty('b2')) {
        this.b2 = initObj.b2
      }
      else {
        this.b2 = 0.0;
      }
      if (initObj.hasOwnProperty('b3')) {
        this.b3 = initObj.b3
      }
      else {
        this.b3 = 0.0;
      }
      if (initObj.hasOwnProperty('b4')) {
        this.b4 = initObj.b4
      }
      else {
        this.b4 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Point
    // Serialize message field [x1]
    bufferOffset = _serializer.float64(obj.x1, buffer, bufferOffset);
    // Serialize message field [y1]
    bufferOffset = _serializer.float64(obj.y1, buffer, bufferOffset);
    // Serialize message field [x2]
    bufferOffset = _serializer.float64(obj.x2, buffer, bufferOffset);
    // Serialize message field [y2]
    bufferOffset = _serializer.float64(obj.y2, buffer, bufferOffset);
    // Serialize message field [a1]
    bufferOffset = _serializer.float64(obj.a1, buffer, bufferOffset);
    // Serialize message field [a2]
    bufferOffset = _serializer.float64(obj.a2, buffer, bufferOffset);
    // Serialize message field [a3]
    bufferOffset = _serializer.float64(obj.a3, buffer, bufferOffset);
    // Serialize message field [a4]
    bufferOffset = _serializer.float64(obj.a4, buffer, bufferOffset);
    // Serialize message field [b1]
    bufferOffset = _serializer.float64(obj.b1, buffer, bufferOffset);
    // Serialize message field [b2]
    bufferOffset = _serializer.float64(obj.b2, buffer, bufferOffset);
    // Serialize message field [b3]
    bufferOffset = _serializer.float64(obj.b3, buffer, bufferOffset);
    // Serialize message field [b4]
    bufferOffset = _serializer.float64(obj.b4, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Point
    let len;
    let data = new Point(null);
    // Deserialize message field [x1]
    data.x1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y1]
    data.y1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [x2]
    data.x2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y2]
    data.y2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [a1]
    data.a1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [a2]
    data.a2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [a3]
    data.a3 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [a4]
    data.a4 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [b1]
    data.b1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [b2]
    data.b2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [b3]
    data.b3 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [b4]
    data.b4 = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 96;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rfans_driver/Point';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c81b1cf288dd128754d09c9e4bbb83cf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #存储距离雷达最近的两对锥桶及其中心点的坐标
    
    float64 x1
    float64 y1
    float64 x2
    float64 y2
    float64 a1
    float64 a2
    float64 a3
    float64 a4
    float64 b1
    float64 b2
    float64 b3
    float64 b4
    
    
    
    
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Point(null);
    if (msg.x1 !== undefined) {
      resolved.x1 = msg.x1;
    }
    else {
      resolved.x1 = 0.0
    }

    if (msg.y1 !== undefined) {
      resolved.y1 = msg.y1;
    }
    else {
      resolved.y1 = 0.0
    }

    if (msg.x2 !== undefined) {
      resolved.x2 = msg.x2;
    }
    else {
      resolved.x2 = 0.0
    }

    if (msg.y2 !== undefined) {
      resolved.y2 = msg.y2;
    }
    else {
      resolved.y2 = 0.0
    }

    if (msg.a1 !== undefined) {
      resolved.a1 = msg.a1;
    }
    else {
      resolved.a1 = 0.0
    }

    if (msg.a2 !== undefined) {
      resolved.a2 = msg.a2;
    }
    else {
      resolved.a2 = 0.0
    }

    if (msg.a3 !== undefined) {
      resolved.a3 = msg.a3;
    }
    else {
      resolved.a3 = 0.0
    }

    if (msg.a4 !== undefined) {
      resolved.a4 = msg.a4;
    }
    else {
      resolved.a4 = 0.0
    }

    if (msg.b1 !== undefined) {
      resolved.b1 = msg.b1;
    }
    else {
      resolved.b1 = 0.0
    }

    if (msg.b2 !== undefined) {
      resolved.b2 = msg.b2;
    }
    else {
      resolved.b2 = 0.0
    }

    if (msg.b3 !== undefined) {
      resolved.b3 = msg.b3;
    }
    else {
      resolved.b3 = 0.0
    }

    if (msg.b4 !== undefined) {
      resolved.b4 = msg.b4;
    }
    else {
      resolved.b4 = 0.0
    }

    return resolved;
    }
};

module.exports = Point;
