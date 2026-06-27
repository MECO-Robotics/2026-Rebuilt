import 'package:flutter/material.dart';

import '../services/dashboard_state.dart';

class GamePiecePanel extends StatelessWidget {
  const GamePiecePanel({required this.dashboardState, super.key});

  final DashboardState dashboardState;

  @override
  Widget build(BuildContext context) {
    final accent =
        dashboardState.seesBalls ? const Color(0xff46d37d) : const Color(0xffffc857);
    return Container(
      padding: const EdgeInsets.all(14),
      decoration: BoxDecoration(
        color: const Color(0xff171c31),
        border: Border.all(color: const Color(0xff303957)),
        borderRadius: BorderRadius.circular(8),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          Row(
            children: [
              Expanded(
                child: Text(
                  'Game Piece Vision',
                  style: Theme.of(context).textTheme.titleMedium?.copyWith(
                        fontWeight: FontWeight.w800,
                      ),
                ),
              ),
              _SignalPill(
                label: dashboardState.gamePieceCameraConnected
                    ? 'Camera Online'
                    : 'Camera Offline',
                color: dashboardState.gamePieceCameraConnected
                    ? const Color(0xff46d37d)
                    : const Color(0xffff4b6f),
              ),
            ],
          ),
          const SizedBox(height: 12),
          SizedBox(
            height: 88,
            child: Row(
              children: [
                Expanded(
                  child: _CameraTile(
                    label: 'Front',
                    streamUrl: dashboardState.frontCameraVideoUrl,
                    dashboardUrl: dashboardState.frontCameraDashboardUrl,
                  ),
                ),
                const SizedBox(width: 10),
                Expanded(
                  child: _CameraTile(
                    label: 'Rear',
                    streamUrl: dashboardState.rearCameraVideoUrl,
                    dashboardUrl: dashboardState.rearCameraDashboardUrl,
                  ),
                ),
              ],
            ),
          ),
          const SizedBox(height: 12),
          Row(
            children: [
              _SignalPill(
                label: dashboardState.seesBalls ? 'Balls Visible' : 'No Balls',
                color: accent,
              ),
              const SizedBox(width: 8),
              _SignalPill(
                label: '${dashboardState.detectedGroups} groups',
                color: const Color(0xff8ea2ff),
              ),
              const SizedBox(width: 8),
              Expanded(
                child: _SignalPill(
                  label: dashboardState.selectedShape.isEmpty
                      ? 'shape --'
                      : dashboardState.selectedShape,
                  color: const Color(0xffc4ccff),
                ),
              ),
            ],
          ),
          const SizedBox(height: 12),
          Row(
            children: [
              Expanded(
                child: _MetricCard(
                  label: 'Selected',
                  value: '${dashboardState.selectedBallCount}',
                  detail:
                      '${_meters(dashboardState.selectedDistanceMeters)}  ${_degrees(dashboardState.selectedYawDegrees)}',
                ),
              ),
              const SizedBox(width: 10),
              Expanded(
                child: _MetricCard(
                  label: 'Biggest',
                  value: '${dashboardState.biggestBallCount}',
                  detail: _meters(dashboardState.biggestDistanceMeters),
                ),
              ),
              const SizedBox(width: 10),
              Expanded(
                child: _MetricCard(
                  label: 'Closest',
                  value: '${dashboardState.closestBallCount}',
                  detail: _meters(dashboardState.closestDistanceMeters),
                ),
              ),
            ],
          ),
          const SizedBox(height: 10),
          Text(
            dashboardState.selectedReason.isEmpty
                ? 'Waiting for selected group...'
                : dashboardState.selectedReason,
            maxLines: 2,
            overflow: TextOverflow.ellipsis,
            style: TextStyle(
              color: const Color(0xffcbd4ff).withValues(alpha: 0.82),
              height: 1.2,
            ),
          ),
        ],
      ),
    );
  }

  static String _meters(double meters) {
    return meters > 0.0 ? '${meters.toStringAsFixed(2)} m' : '-- m';
  }

  static String _degrees(double degrees) {
    return '${degrees.toStringAsFixed(1)} deg';
  }
}

class _CameraTile extends StatelessWidget {
  const _CameraTile({
    required this.label,
    required this.streamUrl,
    required this.dashboardUrl,
  });

  final String label;
  final String streamUrl;
  final String dashboardUrl;

  @override
  Widget build(BuildContext context) {
    return ClipRRect(
      borderRadius: BorderRadius.circular(8),
      child: Stack(
        fit: StackFit.expand,
        children: [
          ColoredBox(
            color: const Color(0xff0c1020),
            child: Image.network(
              streamUrl,
              gaplessPlayback: true,
              fit: BoxFit.cover,
              errorBuilder: (context, error, stackTrace) {
                return Center(
                  child: Padding(
                    padding: const EdgeInsets.all(10),
                    child: Text(
                      dashboardUrl,
                      textAlign: TextAlign.center,
                      overflow: TextOverflow.ellipsis,
                      maxLines: 3,
                      style: const TextStyle(
                        color: Color(0xff9aa7d6),
                        fontSize: 11,
                      ),
                    ),
                  ),
                );
              },
            ),
          ),
          Positioned(
            left: 8,
            top: 8,
            child: _SignalPill(label: label, color: const Color(0xfff0f3ff)),
          ),
        ],
      ),
    );
  }
}

class _MetricCard extends StatelessWidget {
  const _MetricCard({
    required this.label,
    required this.value,
    required this.detail,
  });

  final String label;
  final String value;
  final String detail;

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 9),
      decoration: BoxDecoration(
        color: const Color(0xff10162a),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: const Color(0xff2b3659)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            label,
            overflow: TextOverflow.ellipsis,
            style: const TextStyle(color: Color(0xff94a1cf), fontSize: 12),
          ),
          const SizedBox(height: 2),
          Text(
            value,
            overflow: TextOverflow.ellipsis,
            style: const TextStyle(
              fontSize: 26,
              fontWeight: FontWeight.w800,
              height: 1.0,
            ),
          ),
          const SizedBox(height: 2),
          Text(
            detail,
            overflow: TextOverflow.ellipsis,
            style: const TextStyle(color: Color(0xffcbd4ff), fontSize: 12),
          ),
        ],
      ),
    );
  }
}

class _SignalPill extends StatelessWidget {
  const _SignalPill({required this.label, required this.color});

  final String label;
  final Color color;

  @override
  Widget build(BuildContext context) {
    return Container(
      height: 30,
      padding: const EdgeInsets.symmetric(horizontal: 10),
      alignment: Alignment.center,
      decoration: BoxDecoration(
        color: color.withValues(alpha: 0.15),
        border: Border.all(color: color.withValues(alpha: 0.55)),
        borderRadius: BorderRadius.circular(8),
      ),
      child: Text(
        label,
        overflow: TextOverflow.ellipsis,
        style: TextStyle(
          color: color,
          fontSize: 12,
          fontWeight: FontWeight.w800,
        ),
      ),
    );
  }
}
