import 'dart:math';

class FieldTarget {
  const FieldTarget({
    required this.name,
    required this.xMeters,
    required this.yMeters,
    required this.headingDegrees,
    required this.group,
  });

  final String name;
  final double xMeters;
  final double yMeters;
  final double headingDegrees;
  final TargetGroup group;

  Point<double> get point => Point<double>(xMeters, yMeters);
}

enum TargetGroup {
  hub,
  trench,
  depot,
  outpost,
  climb,
  bump,
  counter,
}

const double fieldLengthMeters = 16.540988;
const double fieldWidthMeters = 8.052;
const double allianceZoneLengthMeters = fieldLengthMeters / 2.0;

const List<FieldTarget> autoDriveTargets = <FieldTarget>[
  FieldTarget(
      name: 'Hub Fender',
      xMeters: 2.960,
      yMeters: 3.938,
      headingDegrees: 0.0,
      group: TargetGroup.hub),
  FieldTarget(
      name: 'Hub Left Face',
      xMeters: 3.180,
      yMeters: 4.980,
      headingDegrees: -28.0,
      group: TargetGroup.hub),
  FieldTarget(
      name: 'Hub Right Face',
      xMeters: 3.180,
      yMeters: 3.090,
      headingDegrees: 28.0,
      group: TargetGroup.hub),
  FieldTarget(
      name: 'Left Trench',
      xMeters: 7.159,
      yMeters: 4.993,
      headingDegrees: -136.0,
      group: TargetGroup.trench),
  FieldTarget(
      name: 'Left Trench Return',
      xMeters: 2.697,
      yMeters: 5.633,
      headingDegrees: -39.8,
      group: TargetGroup.trench),
  FieldTarget(
      name: 'Left Trench Far',
      xMeters: 7.892,
      yMeters: 5.944,
      headingDegrees: 130.4,
      group: TargetGroup.trench),
  FieldTarget(
      name: 'Left Trench Corner',
      xMeters: 2.549,
      yMeters: 7.229,
      headingDegrees: -57.8,
      group: TargetGroup.trench),
  FieldTarget(
      name: 'Left Trench Cleanup',
      xMeters: 8.292,
      yMeters: 5.342,
      headingDegrees: 8.7,
      group: TargetGroup.trench),
  FieldTarget(
      name: 'Right Trench',
      xMeters: 7.159,
      yMeters: 3.077,
      headingDegrees: 136.0,
      group: TargetGroup.trench),
  FieldTarget(
      name: 'Right Trench Return',
      xMeters: 2.234,
      yMeters: 2.416,
      headingDegrees: 35.0,
      group: TargetGroup.trench),
  FieldTarget(
      name: 'Right Trench Far',
      xMeters: 7.173,
      yMeters: 2.416,
      headingDegrees: -178.8,
      group: TargetGroup.trench),
  FieldTarget(
      name: 'Right Trench Corner',
      xMeters: 2.611,
      yMeters: 2.480,
      headingDegrees: 37.6,
      group: TargetGroup.trench),
  FieldTarget(
      name: 'Right Trench Cleanup',
      xMeters: 8.292,
      yMeters: 2.728,
      headingDegrees: -8.7,
      group: TargetGroup.trench),
  FieldTarget(
      name: 'Left Depot',
      xMeters: 3.333,
      yMeters: 2.670,
      headingDegrees: 45.0,
      group: TargetGroup.depot),
  FieldTarget(
      name: 'Right Depot',
      xMeters: 3.333,
      yMeters: 5.400,
      headingDegrees: -45.0,
      group: TargetGroup.depot),
  FieldTarget(
      name: 'Left Rush Start',
      xMeters: 3.333,
      yMeters: 5.364,
      headingDegrees: -45.0,
      group: TargetGroup.depot),
  FieldTarget(
      name: 'Right Rush Start',
      xMeters: 3.333,
      yMeters: 2.706,
      headingDegrees: 45.0,
      group: TargetGroup.depot),
  FieldTarget(
      name: 'Depot Pickup',
      xMeters: 1.820,
      yMeters: 1.120,
      headingDegrees: 0.0,
      group: TargetGroup.depot),
  FieldTarget(
      name: 'Outpost Center',
      xMeters: 7.440,
      yMeters: 4.026,
      headingDegrees: 180.0,
      group: TargetGroup.outpost),
  FieldTarget(
      name: 'Outpost Left',
      xMeters: 7.230,
      yMeters: 5.000,
      headingDegrees: -150.0,
      group: TargetGroup.outpost),
  FieldTarget(
      name: 'Outpost Right',
      xMeters: 7.230,
      yMeters: 3.050,
      headingDegrees: 150.0,
      group: TargetGroup.outpost),
  FieldTarget(
      name: 'Climb Center',
      xMeters: 5.180,
      yMeters: 4.026,
      headingDegrees: 0.0,
      group: TargetGroup.climb),
  FieldTarget(
      name: 'Climb Left',
      xMeters: 5.000,
      yMeters: 5.160,
      headingDegrees: -90.0,
      group: TargetGroup.climb),
  FieldTarget(
      name: 'Climb Right',
      xMeters: 5.000,
      yMeters: 2.890,
      headingDegrees: 90.0,
      group: TargetGroup.climb),
  FieldTarget(
      name: 'Left Bump',
      xMeters: 3.206,
      yMeters: 5.615,
      headingDegrees: -89.3,
      group: TargetGroup.bump),
  FieldTarget(
      name: 'Left Bump Far',
      xMeters: 7.922,
      yMeters: 6.522,
      headingDegrees: 131.5,
      group: TargetGroup.bump),
  FieldTarget(
      name: 'Right Bump',
      xMeters: 7.159,
      yMeters: 3.077,
      headingDegrees: 136.0,
      group: TargetGroup.bump),
  FieldTarget(
      name: 'Right Bump Near',
      xMeters: 2.234,
      yMeters: 2.416,
      headingDegrees: 35.0,
      group: TargetGroup.bump),
  FieldTarget(
      name: 'Right Bump Far',
      xMeters: 7.173,
      yMeters: 2.416,
      headingDegrees: -178.8,
      group: TargetGroup.bump),
  FieldTarget(
      name: 'Right Bump Corner',
      xMeters: 2.611,
      yMeters: 2.480,
      headingDegrees: 37.6,
      group: TargetGroup.bump),
  FieldTarget(
      name: 'Right Bump Cleanup',
      xMeters: 8.292,
      yMeters: 2.728,
      headingDegrees: -8.7,
      group: TargetGroup.bump),
  FieldTarget(
      name: 'Left Prom Counter',
      xMeters: 2.985,
      yMeters: 5.806,
      headingDegrees: -47.9,
      group: TargetGroup.counter),
  FieldTarget(
      name: 'Left Prom Counter Send',
      xMeters: 7.678,
      yMeters: 4.686,
      headingDegrees: -41.8,
      group: TargetGroup.counter),
  FieldTarget(
      name: 'Left Prom Counter Cleanup',
      xMeters: 5.911,
      yMeters: 4.064,
      headingDegrees: -110.0,
      group: TargetGroup.counter),
  FieldTarget(
      name: 'Right Prom Counter',
      xMeters: 2.985,
      yMeters: 2.264,
      headingDegrees: 47.9,
      group: TargetGroup.counter),
  FieldTarget(
      name: 'Right Prom Counter Send',
      xMeters: 7.678,
      yMeters: 3.384,
      headingDegrees: 41.8,
      group: TargetGroup.counter),
  FieldTarget(
      name: 'Right Prom Counter Cleanup',
      xMeters: 5.911,
      yMeters: 4.006,
      headingDegrees: 110.0,
      group: TargetGroup.counter),
];
