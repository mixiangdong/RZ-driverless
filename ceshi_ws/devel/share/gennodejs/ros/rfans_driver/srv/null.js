// Auto-generated. Do not edit!

// (in-package rfans_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class nullRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.state = null;
    }
    else {
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type nullRequest
    // Serialize message field [state]
    bufferOffset = _serializer.bool(obj.state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type nullRequest
    let len;
    let data = new nullRequest(null);
    // Deserialize message field [state]
    data.state = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rfans_driver/nullRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '001fde3cab9e313a150416ff09c08ee4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    bool state
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new nullRequest(null);
    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = false
    }

    return resolved;
    }
};

class nullResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.work = null;
    }
    else {
      if (initObj.hasOwnProperty('work')) {
        this.work = initObj.work
      }
      else {
        this.work = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type nullResponse
    // Serialize message field [work]
    bufferOffset = _serializer.bool(obj.work, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type nullResponse
    let len;
    let data = new nullResponse(null);
    // Deserialize message field [work]
    data.work = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rfans_driver/nullResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'eed3ad19ae1523e96be34bf5a1ab27e3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool work
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new nullResponse(null);
    if (msg.work !== undefined) {
      resolved.work = msg.work;
    }
    else {
      resolved.work = false
    }

    return resolved;
    }
};

module.exports = {
  Request: nullRequest,
  Response: nullResponse,
  md5sum() { return '0ec340068269569bdcfcbf79cb9781d5'; },
  datatype() { return 'rfans_driver/null'; }
};
