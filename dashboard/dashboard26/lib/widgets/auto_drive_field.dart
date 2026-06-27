import 'dart:math' as math;

import 'package:flutter/material.dart';

import '../models/field_target.dart';

class AutoDriveField extends StatefulWidget {
  const AutoDriveField({
    required this.targets,
    required this.redAlliance,
    required this.onPoseSelected,
    super.key,
  });

  final List<FieldTarget> targets;
  final bool redAlliance;
  final void Function(double xMeters, double yMeters, double headingDegrees)
      onPoseSelected;

  @override
  State<AutoDriveField> createState() => _AutoDriveFieldState();
}

class _AutoDriveFieldState extends State<AutoDriveField> {
  FieldTarget? selectedTarget;
  Offset? lastClick;

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(
      builder: (context, constraints) {
        final fieldSize = _fieldSize(constraints.biggest);
        final origin = Offset(
          (constraints.maxWidth - fieldSize.width) / 2.0,
          (constraints.maxHeight - fieldSize.height) / 2.0,
        );
        return GestureDetector(
          behavior: HitTestBehavior.opaque,
          onTapUp: (details) {
            final local = details.localPosition - origin;
            if (!_inside(local, fieldSize)) {
              return;
            }
            final target = _nearestTarget(local, fieldSize);
            if (target != null) {
              setState(() {
                selectedTarget = target;
                lastClick = _targetToOffset(target, fieldSize);
              });
              final pose = _targetToPose(target);
              widget.onPoseSelected(pose.$1, pose.$2, pose.$3);
              return;
            }

            final pose = _offsetToPose(local, fieldSize);
            setState(() {
              selectedTarget = null;
              lastClick = local;
            });
            widget.onPoseSelected(
                pose.$1, pose.$2, widget.redAlliance ? 180.0 : 0.0);
          },
          child: CustomPaint(
            size: constraints.biggest,
            painter: _FieldPainter(
              fieldSize: fieldSize,
              origin: origin,
              targets: widget.targets,
              selectedTarget: selectedTarget,
              lastClick: lastClick,
              redAlliance: widget.redAlliance,
            ),
          ),
        );
      },
    );
  }

  Size _fieldSize(Size available) {
    const aspect = allianceZoneLengthMeters / fieldWidthMeters;
    var width = available.width;
    var height = width / aspect;
    if (height > available.height) {
      height = available.height;
      width = height * aspect;
    }
    return Size(width, height);
  }

  bool _inside(Offset offset, Size fieldSize) {
    return offset.dx >= 0.0 &&
        offset.dy >= 0.0 &&
        offset.dx <= fieldSize.width &&
        offset.dy <= fieldSize.height;
  }

  FieldTarget? _nearestTarget(Offset offset, Size fieldSize) {
    FieldTarget? nearest;
    var nearestDistance = double.infinity;
    for (final target in widget.targets) {
      final targetOffset = _targetToOffset(target, fieldSize);
      final distance = (targetOffset - offset).distance;
      if (distance < nearestDistance) {
        nearest = target;
        nearestDistance = distance;
      }
    }
    return nearestDistance <= 34.0 ? nearest : null;
  }

  Offset _targetToOffset(FieldTarget target, Size fieldSize) {
    final allianceX =
        target.xMeters.clamp(0.0, allianceZoneLengthMeters).toDouble();
    return Offset(
      (allianceX / allianceZoneLengthMeters) * fieldSize.width,
      fieldSize.height - (target.yMeters / fieldWidthMeters) * fieldSize.height,
    );
  }

  (double, double, double) _targetToPose(FieldTarget target) {
    if (!widget.redAlliance) {
      return (target.xMeters, target.yMeters, target.headingDegrees);
    }
    return (
      fieldLengthMeters - target.xMeters,
      target.yMeters,
      _wrapDegrees(180.0 - target.headingDegrees),
    );
  }

  (double, double) _offsetToPose(Offset offset, Size fieldSize) {
    final allianceX = (offset.dx / fieldSize.width) * allianceZoneLengthMeters;
    final fieldX =
        widget.redAlliance ? fieldLengthMeters - allianceX : allianceX;
    final fieldY =
        fieldWidthMeters - (offset.dy / fieldSize.height) * fieldWidthMeters;
    return (
      fieldX.clamp(0.0, fieldLengthMeters).toDouble(),
      fieldY.clamp(0.0, fieldWidthMeters).toDouble(),
    );
  }

  double _wrapDegrees(double degrees) {
    var wrapped = degrees;
    while (wrapped > 180.0) {
      wrapped -= 360.0;
    }
    while (wrapped <= -180.0) {
      wrapped += 360.0;
    }
    return wrapped;
  }
}

class _FieldPainter extends CustomPainter {
  const _FieldPainter({
    required this.fieldSize,
    required this.origin,
    required this.targets,
    required this.selectedTarget,
    required this.lastClick,
    required this.redAlliance,
  });

  final Size fieldSize;
  final Offset origin;
  final List<FieldTarget> targets;
  final FieldTarget? selectedTarget;
  final Offset? lastClick;
  final bool redAlliance;

  static const double _fieldWidthInches = 317.7;
  static const double _hubSizeInches = 47.0;
  static const double _bumpDepthInches = 44.4;
  static const double _trenchDepthInches = 47.0;
  static const double _depotWidthInches = 42.0;
  static const double _depotDepthInches = 27.0;
  static const double _towerWidthInches = 49.25;
  static const double _towerDepthInches = 45.0;
  static const double _outpostOpeningWidthInches = 31.8;
  static const double _outpostDepthInches = 14.0;

  Rect get zoneRect => Rect.fromLTWH(
        fieldSize.width * 0.05,
        fieldSize.height * 0.14,
        fieldSize.width * 0.90,
        fieldSize.height * 0.72,
      );

  double get _pxPerInch => zoneRect.width / _fieldWidthInches;

  @override
  void paint(Canvas canvas, Size size) {
    canvas.save();
    canvas.translate(origin.dx, origin.dy);
    _drawField(canvas);
    _drawTargets(canvas);
    canvas.restore();
  }

  void _drawField(Canvas canvas) {
    final bounds = Offset.zero & fieldSize;
    final background = Paint()
      ..shader = const LinearGradient(
        begin: Alignment.topLeft,
        end: Alignment.bottomRight,
        colors: [Color(0xff171b32), Color(0xff0e1428)],
      ).createShader(bounds);
    final allianceGlow = Paint()
      ..color =
          (redAlliance ? const Color(0xffff4164) : const Color(0xff5578ff))
              .withValues(alpha: 0.08);
    final frame = Paint()
      ..color = const Color(0xffb8c2ff).withValues(alpha: 0.42)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 3.0;

    canvas.drawRRect(
        RRect.fromRectAndRadius(bounds, const Radius.circular(8)), background);
    canvas.drawRRect(
        RRect.fromRectAndRadius(bounds.deflate(1), const Radius.circular(8)),
        allianceGlow);

    _drawAllianceZoneShell(canvas);
    _drawTrenchBumpHubBand(canvas);
    _drawLowerStructures(canvas);
    _drawFieldEdges(canvas);
    canvas.drawRRect(
        RRect.fromRectAndRadius(bounds.deflate(2), const Radius.circular(8)),
        frame);
  }

  void _drawAllianceZoneShell(Canvas canvas) {
    final line = Paint()
      ..color = const Color(0xffcbd4ff).withValues(alpha: 0.28)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.0
      ..strokeCap = StrokeCap.round;
    final zoneFill = Paint()
      ..color = const Color(0xff7c87cf).withValues(alpha: 0.07)
      ..style = PaintingStyle.fill;

    final zone = RRect.fromRectAndRadius(zoneRect, const Radius.circular(12));

    canvas.drawRRect(zone, zoneFill);
    canvas.drawRRect(zone, line);

    canvas.drawLine(
      Offset(fieldSize.width * 0.50, fieldSize.height * 0.09),
      Offset(fieldSize.width * 0.50, fieldSize.height * 0.91),
      line,
    );
  }

  void _drawTrenchBumpHubBand(Canvas canvas) {
    final zone = zoneRect;
    final bandCenterY = zone.top + zone.height * 0.24;
    final bumpDepth = _inches(_bumpDepthInches);
    final trenchDepth = _inches(_trenchDepthInches);
    final hubSize = _inches(_hubSizeInches);
    final hubCenter = Offset(zone.center.dx, bandCenterY);
    final hubStructureRect = _hubStructureRect(hubCenter, hubSize, bumpDepth);
    final leftBumpRight = hubStructureRect.left;
    final rightBumpLeft = hubStructureRect.right;
    final bumpWidth = (leftBumpRight - zone.left) * 0.48;
    final trenchPaint = Paint()
      ..color = const Color(0xff6874ae).withValues(alpha: 0.24)
      ..style = PaintingStyle.fill;
    final bumpPaint = Paint()
      ..color =
          (redAlliance ? const Color(0xffff4164) : const Color(0xff5578ff))
              .withValues(alpha: 0.42)
      ..style = PaintingStyle.fill;
    final outline = Paint()
      ..color = const Color(0xffdce2ff).withValues(alpha: 0.55)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.6
      ..strokeCap = StrokeCap.round;

    final leftTrench = Rect.fromLTWH(
      zone.left,
      bandCenterY - trenchDepth / 2,
      leftBumpRight - bumpWidth - zone.left,
      trenchDepth,
    );
    final leftBump = Rect.fromLTWH(
      leftBumpRight - bumpWidth,
      bandCenterY - bumpDepth / 2,
      bumpWidth,
      bumpDepth,
    );
    final rightBump = Rect.fromLTWH(
      rightBumpLeft,
      bandCenterY - bumpDepth / 2,
      bumpWidth,
      bumpDepth,
    );
    final rightTrench = Rect.fromLTWH(
      rightBump.right,
      bandCenterY - trenchDepth / 2,
      zone.right - rightBump.right,
      trenchDepth,
    );

    _drawSoftRect(canvas, leftTrench, trenchPaint, outline, 7);
    _drawSoftRect(canvas, leftBump, bumpPaint, outline, 7);
    _drawSoftRect(canvas, rightBump, bumpPaint, outline, 7);
    _drawSoftRect(canvas, rightTrench, trenchPaint, outline, 7);
    _drawHubGlyph(canvas, hubCenter, hubSize, bumpDepth);
    _drawSoftLabel(canvas, 'Trench', leftTrench.center + const Offset(0, 38));
    _drawSoftLabel(canvas, 'Bump', leftBump.center);
    _drawSoftLabel(canvas, 'Bump', rightBump.center);
    _drawSoftLabel(canvas, 'Trench', rightTrench.center + const Offset(0, 38));
  }

  void _drawLowerStructures(Canvas canvas) {
    _drawTower(canvas);
    _drawDepot(canvas);
    _drawOutpost(canvas);
  }

  void _drawTower(Canvas canvas) {
    final towerRect = Rect.fromLTWH(
      zoneRect.center.dx - _inches(_towerWidthInches) / 2,
      zoneRect.bottom - _inches(_towerDepthInches),
      _inches(_towerWidthInches),
      _inches(_towerDepthInches),
    );
    final glow = Paint()
      ..color = const Color(0xffffcf33).withValues(alpha: 0.12)
      ..style = PaintingStyle.fill
      ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 14);
    final fill = Paint()
      ..color = const Color(0xffffcf33).withValues(alpha: 0.16)
      ..style = PaintingStyle.fill;
    final stroke = Paint()
      ..color = const Color(0xffffd400).withValues(alpha: 0.84)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 4.0
      ..strokeCap = StrokeCap.round
      ..strokeJoin = StrokeJoin.round;
    final innerStroke = Paint()
      ..color = const Color(0xfffff0a8).withValues(alpha: 0.36)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.0;

    canvas.drawRRect(
      RRect.fromRectAndRadius(towerRect.inflate(8), const Radius.circular(10)),
      glow,
    );
    _drawSoftRect(canvas, towerRect, fill, stroke, 8);
    canvas.drawLine(towerRect.topCenter, towerRect.bottomCenter, innerStroke);
    canvas.drawLine(towerRect.centerLeft, towerRect.centerRight, innerStroke);
    _drawSoftLabel(canvas, 'Climb', towerRect.center);
  }

  void _drawDepot(Canvas canvas) {
    final depotRect = Rect.fromLTWH(
      zoneRect.right - _inches(72.0),
      zoneRect.bottom - _inches(_depotDepthInches),
      _inches(_depotWidthInches),
      _inches(_depotDepthInches),
    );
    final fill = Paint()
      ..color = const Color(0xff6d78b8).withValues(alpha: 0.26)
      ..style = PaintingStyle.fill;
    final stroke = Paint()
      ..color = const Color(0xffe4e8ff).withValues(alpha: 0.50)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.6;

    _drawSoftRect(canvas, depotRect, fill, stroke, 8);
    _drawSoftLabel(canvas, 'Depot', depotRect.center);
  }

  void _drawOutpost(Canvas canvas) {
    final outpostRect = Rect.fromLTWH(
      zoneRect.left + _inches(38.0),
      zoneRect.bottom - _inches(_outpostDepthInches),
      _inches(_outpostOpeningWidthInches),
      _inches(_outpostDepthInches),
    );
    final fill = Paint()
      ..color = const Color(0xff27d17f).withValues(alpha: 0.18)
      ..style = PaintingStyle.fill;
    final stroke = Paint()
      ..color = const Color(0xff29d17f).withValues(alpha: 0.78)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 3.2;

    _drawSoftRect(canvas, outpostRect, fill, stroke, 6);
    _drawSoftLabel(
        canvas, 'Outpost', outpostRect.center + const Offset(0, -30));
  }

  void _drawHubGlyph(
      Canvas canvas, Offset center, double hubSize, double bandDepth) {
    final radius = hubSize / 2;
    final structureRect = _hubStructureRect(center, hubSize, bandDepth);
    final glow = Paint()
      ..color = const Color(0xffdce2ff).withValues(alpha: 0.16)
      ..style = PaintingStyle.fill
      ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 10);
    final fill = Paint()
      ..color = const Color(0xffc8cffd).withValues(alpha: 0.76)
      ..style = PaintingStyle.fill;
    final stroke = Paint()
      ..color = const Color(0xffffffff).withValues(alpha: 0.68)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 3.0;
    final structureFill = Paint()
      ..color = const Color(0xffc8cffd).withValues(alpha: 0.12)
      ..style = PaintingStyle.fill;
    final structureStroke = Paint()
      ..color = const Color(0xfff4f6ff).withValues(alpha: 0.52)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.4;
    final path = Path();

    canvas.drawCircle(center, radius * 1.45, glow);
    _drawSoftRect(canvas, structureRect, structureFill, structureStroke, 2);
    for (var i = 0; i < 6; i += 1) {
      final angle = math.pi / 6.0 + i * math.pi / 3.0;
      final point = Offset(
        center.dx + math.cos(angle) * radius,
        center.dy + math.sin(angle) * radius,
      );
      if (i == 0) {
        path.moveToPoint(point);
      } else {
        path.lineToPoint(point);
      }
    }
    path.close();
    canvas.drawPath(path, fill);
    canvas.drawPath(path, stroke);
  }

  Rect _hubStructureRect(Offset center, double hubSize, double bandDepth) {
    return Rect.fromCenter(
      center: center,
      width: hubSize * 1.44,
      height: bandDepth,
    );
  }

  void _drawFieldEdges(Canvas canvas) {
    final wallPaint = Paint()
      ..color = const Color(0xffffffff).withValues(alpha: 0.58)
      ..strokeWidth = 5.0;
    final centerLinePaint = Paint()
      ..color = const Color(0xffffffff).withValues(alpha: 0.22)
      ..strokeWidth = 4.0;

    canvas.drawLine(Offset.zero, Offset(0, fieldSize.height), wallPaint);
    canvas.drawLine(Offset(fieldSize.width, 0),
        Offset(fieldSize.width, fieldSize.height), centerLinePaint);
  }

  void _drawSoftRect(
    Canvas canvas,
    Rect rect,
    Paint fill,
    Paint stroke,
    double radius,
  ) {
    final rrect = RRect.fromRectAndRadius(rect, Radius.circular(radius));
    canvas.drawRRect(rrect, fill);
    canvas.drawRRect(rrect, stroke);
  }

  void _drawSoftLabel(Canvas canvas, String label, Offset center) {
    final textPainter = TextPainter(
      text: TextSpan(
        text: label,
        style: TextStyle(
          color: Colors.white.withValues(alpha: 0.78),
          fontSize: 18,
          fontWeight: FontWeight.w500,
          letterSpacing: 0,
        ),
      ),
      textAlign: TextAlign.center,
      textDirection: TextDirection.ltr,
    )..layout(maxWidth: 130);
    textPainter.paint(
      canvas,
      center - Offset(textPainter.width / 2.0, textPainter.height / 2.0),
    );
  }

  double _inches(double inches) => inches * _pxPerInch;

  void _drawTargets(Canvas canvas) {
    final fillPaint = Paint()
      ..style = PaintingStyle.fill
      ..color = const Color(0xffff4268).withValues(alpha: 0.92);
    final clickPaint = Paint()
      ..style = PaintingStyle.fill
      ..color = const Color(0xffff4268).withValues(alpha: 0.92);

    for (final target in targets) {
      final offset = _targetToOffset(target);
      final selected = selectedTarget == target;
      if (selected) {
        canvas.drawCircle(offset, 30, fillPaint);
        _drawCheck(canvas, offset);
      }
    }

    if (lastClick != null && selectedTarget == null) {
      canvas.drawCircle(lastClick!, 29, clickPaint);
      _drawCheck(canvas, lastClick!);
    }
  }

  void _drawCheck(Canvas canvas, Offset center) {
    final paint = Paint()
      ..color = Colors.white
      ..strokeWidth = 7.0
      ..strokeCap = StrokeCap.round
      ..style = PaintingStyle.stroke;
    final path = Path()
      ..moveTo(center.dx - 12, center.dy - 1)
      ..lineTo(center.dx - 3, center.dy + 10)
      ..lineTo(center.dx + 15, center.dy - 14);
    canvas.drawPath(path, paint);
  }

  Offset _targetToOffset(FieldTarget target) {
    final allianceX =
        target.xMeters.clamp(0.0, allianceZoneLengthMeters).toDouble();
    return Offset(
      (allianceX / allianceZoneLengthMeters) * fieldSize.width,
      fieldSize.height - (target.yMeters / fieldWidthMeters) * fieldSize.height,
    );
  }

  @override
  bool shouldRepaint(covariant _FieldPainter oldDelegate) {
    return oldDelegate.targets != targets ||
        oldDelegate.selectedTarget != selectedTarget ||
        oldDelegate.lastClick != lastClick ||
        oldDelegate.redAlliance != redAlliance ||
        oldDelegate.fieldSize != fieldSize ||
        oldDelegate.origin != origin;
  }
}

extension on Path {
  void moveToPoint(Offset point) => moveTo(point.dx, point.dy);

  void lineToPoint(Offset point) => lineTo(point.dx, point.dy);
}
