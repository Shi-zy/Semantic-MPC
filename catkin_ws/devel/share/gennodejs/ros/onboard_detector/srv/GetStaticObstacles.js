// Auto-generated. Do not edit!

// (in-package onboard_detector.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class GetStaticObstaclesRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.current_position = null;
      this.range = null;
      this.semantic_filter = null;
    }
    else {
      if (initObj.hasOwnProperty('current_position')) {
        this.current_position = initObj.current_position
      }
      else {
        this.current_position = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('range')) {
        this.range = initObj.range
      }
      else {
        this.range = 0.0;
      }
      if (initObj.hasOwnProperty('semantic_filter')) {
        this.semantic_filter = initObj.semantic_filter
      }
      else {
        this.semantic_filter = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetStaticObstaclesRequest
    // Serialize message field [current_position]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.current_position, buffer, bufferOffset);
    // Serialize message field [range]
    bufferOffset = _serializer.float64(obj.range, buffer, bufferOffset);
    // Serialize message field [semantic_filter]
    bufferOffset = _serializer.string(obj.semantic_filter, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetStaticObstaclesRequest
    let len;
    let data = new GetStaticObstaclesRequest(null);
    // Deserialize message field [current_position]
    data.current_position = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [range]
    data.range = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [semantic_filter]
    data.semantic_filter = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.semantic_filter);
    return length + 36;
  }

  static datatype() {
    // Returns string type for a service object
    return 'onboard_detector/GetStaticObstaclesRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'feeefa6eaed2d913d83da97e14fa99bc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Request
    geometry_msgs/Point current_position
    float64 range
    string semantic_filter  # Optional filter by semantic class (e.g., "WALL", "FURNITURE", empty for all)
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetStaticObstaclesRequest(null);
    if (msg.current_position !== undefined) {
      resolved.current_position = geometry_msgs.msg.Point.Resolve(msg.current_position)
    }
    else {
      resolved.current_position = new geometry_msgs.msg.Point()
    }

    if (msg.range !== undefined) {
      resolved.range = msg.range;
    }
    else {
      resolved.range = 0.0
    }

    if (msg.semantic_filter !== undefined) {
      resolved.semantic_filter = msg.semantic_filter;
    }
    else {
      resolved.semantic_filter = ''
    }

    return resolved;
    }
};

class GetStaticObstaclesResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.names = null;
      this.ids = null;
      this.positions = null;
      this.sizes = null;
      this.yaws = null;
      this.semantic_classes = null;
      this.cost_weights = null;
      this.safety_distances = null;
    }
    else {
      if (initObj.hasOwnProperty('names')) {
        this.names = initObj.names
      }
      else {
        this.names = [];
      }
      if (initObj.hasOwnProperty('ids')) {
        this.ids = initObj.ids
      }
      else {
        this.ids = [];
      }
      if (initObj.hasOwnProperty('positions')) {
        this.positions = initObj.positions
      }
      else {
        this.positions = [];
      }
      if (initObj.hasOwnProperty('sizes')) {
        this.sizes = initObj.sizes
      }
      else {
        this.sizes = [];
      }
      if (initObj.hasOwnProperty('yaws')) {
        this.yaws = initObj.yaws
      }
      else {
        this.yaws = [];
      }
      if (initObj.hasOwnProperty('semantic_classes')) {
        this.semantic_classes = initObj.semantic_classes
      }
      else {
        this.semantic_classes = [];
      }
      if (initObj.hasOwnProperty('cost_weights')) {
        this.cost_weights = initObj.cost_weights
      }
      else {
        this.cost_weights = [];
      }
      if (initObj.hasOwnProperty('safety_distances')) {
        this.safety_distances = initObj.safety_distances
      }
      else {
        this.safety_distances = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetStaticObstaclesResponse
    // Serialize message field [names]
    bufferOffset = _arraySerializer.string(obj.names, buffer, bufferOffset, null);
    // Serialize message field [ids]
    bufferOffset = _arraySerializer.uint32(obj.ids, buffer, bufferOffset, null);
    // Serialize message field [positions]
    // Serialize the length for message field [positions]
    bufferOffset = _serializer.uint32(obj.positions.length, buffer, bufferOffset);
    obj.positions.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Vector3.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [sizes]
    // Serialize the length for message field [sizes]
    bufferOffset = _serializer.uint32(obj.sizes.length, buffer, bufferOffset);
    obj.sizes.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Vector3.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [yaws]
    bufferOffset = _arraySerializer.float64(obj.yaws, buffer, bufferOffset, null);
    // Serialize message field [semantic_classes]
    bufferOffset = _arraySerializer.string(obj.semantic_classes, buffer, bufferOffset, null);
    // Serialize message field [cost_weights]
    bufferOffset = _arraySerializer.float64(obj.cost_weights, buffer, bufferOffset, null);
    // Serialize message field [safety_distances]
    bufferOffset = _arraySerializer.float64(obj.safety_distances, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetStaticObstaclesResponse
    let len;
    let data = new GetStaticObstaclesResponse(null);
    // Deserialize message field [names]
    data.names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [ids]
    data.ids = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [positions]
    // Deserialize array length for message field [positions]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.positions = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.positions[i] = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [sizes]
    // Deserialize array length for message field [sizes]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.sizes = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.sizes[i] = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [yaws]
    data.yaws = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [semantic_classes]
    data.semantic_classes = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [cost_weights]
    data.cost_weights = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [safety_distances]
    data.safety_distances = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.names.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    length += 4 * object.ids.length;
    length += 24 * object.positions.length;
    length += 24 * object.sizes.length;
    length += 8 * object.yaws.length;
    object.semantic_classes.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    length += 8 * object.cost_weights.length;
    length += 8 * object.safety_distances.length;
    return length + 32;
  }

  static datatype() {
    // Returns string type for a service object
    return 'onboard_detector/GetStaticObstaclesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '496591114641469de96e821019d3d5c3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Response
    # Array of static obstacles using standard ROS messages
    string[] names
    uint32[] ids
    geometry_msgs/Vector3[] positions
    geometry_msgs/Vector3[] sizes
    float64[] yaws
    string[] semantic_classes
    float64[] cost_weights
    float64[] safety_distances 
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetStaticObstaclesResponse(null);
    if (msg.names !== undefined) {
      resolved.names = msg.names;
    }
    else {
      resolved.names = []
    }

    if (msg.ids !== undefined) {
      resolved.ids = msg.ids;
    }
    else {
      resolved.ids = []
    }

    if (msg.positions !== undefined) {
      resolved.positions = new Array(msg.positions.length);
      for (let i = 0; i < resolved.positions.length; ++i) {
        resolved.positions[i] = geometry_msgs.msg.Vector3.Resolve(msg.positions[i]);
      }
    }
    else {
      resolved.positions = []
    }

    if (msg.sizes !== undefined) {
      resolved.sizes = new Array(msg.sizes.length);
      for (let i = 0; i < resolved.sizes.length; ++i) {
        resolved.sizes[i] = geometry_msgs.msg.Vector3.Resolve(msg.sizes[i]);
      }
    }
    else {
      resolved.sizes = []
    }

    if (msg.yaws !== undefined) {
      resolved.yaws = msg.yaws;
    }
    else {
      resolved.yaws = []
    }

    if (msg.semantic_classes !== undefined) {
      resolved.semantic_classes = msg.semantic_classes;
    }
    else {
      resolved.semantic_classes = []
    }

    if (msg.cost_weights !== undefined) {
      resolved.cost_weights = msg.cost_weights;
    }
    else {
      resolved.cost_weights = []
    }

    if (msg.safety_distances !== undefined) {
      resolved.safety_distances = msg.safety_distances;
    }
    else {
      resolved.safety_distances = []
    }

    return resolved;
    }
};

module.exports = {
  Request: GetStaticObstaclesRequest,
  Response: GetStaticObstaclesResponse,
  md5sum() { return '720433b0b9d7c4c8bbc6acba25184779'; },
  datatype() { return 'onboard_detector/GetStaticObstacles'; }
};
