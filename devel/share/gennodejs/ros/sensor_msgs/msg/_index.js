
"use strict";

let BatteryState = require('./BatteryState.js');
let JointState = require('./JointState.js');
let JoyFeedback = require('./JoyFeedback.js');
let NavSatFix = require('./NavSatFix.js');
let LaserEcho = require('./LaserEcho.js');
let PointField = require('./PointField.js');
let PointCloud = require('./PointCloud.js');
let RelativeHumidity = require('./RelativeHumidity.js');
let NavSatStatus = require('./NavSatStatus.js');
let RegionOfInterest = require('./RegionOfInterest.js');
let PointCloud2 = require('./PointCloud2.js');
let JoyFeedbackArray = require('./JoyFeedbackArray.js');
let Joy = require('./Joy.js');
let Range = require('./Range.js');
let MagneticField = require('./MagneticField.js');
let LaserScan = require('./LaserScan.js');
let MultiDOFJointState = require('./MultiDOFJointState.js');
let MultiEchoLaserScan = require('./MultiEchoLaserScan.js');
let FluidPressure = require('./FluidPressure.js');
let Illuminance = require('./Illuminance.js');
let Imu = require('./Imu.js');
let ChannelFloat32 = require('./ChannelFloat32.js');
let CameraInfo = require('./CameraInfo.js');
let Temperature = require('./Temperature.js');
let TimeReference = require('./TimeReference.js');
let CompressedImage = require('./CompressedImage.js');
let Image = require('./Image.js');

module.exports = {
  BatteryState: BatteryState,
  JointState: JointState,
  JoyFeedback: JoyFeedback,
  NavSatFix: NavSatFix,
  LaserEcho: LaserEcho,
  PointField: PointField,
  PointCloud: PointCloud,
  RelativeHumidity: RelativeHumidity,
  NavSatStatus: NavSatStatus,
  RegionOfInterest: RegionOfInterest,
  PointCloud2: PointCloud2,
  JoyFeedbackArray: JoyFeedbackArray,
  Joy: Joy,
  Range: Range,
  MagneticField: MagneticField,
  LaserScan: LaserScan,
  MultiDOFJointState: MultiDOFJointState,
  MultiEchoLaserScan: MultiEchoLaserScan,
  FluidPressure: FluidPressure,
  Illuminance: Illuminance,
  Imu: Imu,
  ChannelFloat32: ChannelFloat32,
  CameraInfo: CameraInfo,
  Temperature: Temperature,
  TimeReference: TimeReference,
  CompressedImage: CompressedImage,
  Image: Image,
};
