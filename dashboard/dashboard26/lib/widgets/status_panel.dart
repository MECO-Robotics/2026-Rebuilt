import 'package:flutter/material.dart';

import '../services/dashboard_state.dart';

class StatusPanel extends StatelessWidget {
  const StatusPanel({required this.dashboardState, super.key});

  final DashboardState dashboardState;

  @override
  Widget build(BuildContext context) {
    final statusColor = dashboardState.connected ? const Color(0xff46d37d) : const Color(0xffff4b6f);
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        Row(
          children: [
            Expanded(
              child: Text(
                dashboardState.matchTime,
                style: const TextStyle(fontSize: 88, height: 0.95, fontWeight: FontWeight.w300),
                textAlign: TextAlign.right,
              ),
            ),
          ],
        ),
        const SizedBox(height: 14),
        Row(
          children: [
            _StatusPill(label: dashboardState.connected ? 'NT Connected' : 'NT Offline', color: statusColor),
            const SizedBox(width: 10),
            _StatusPill(label: dashboardState.isRedAlliance ? 'Red' : 'Blue', color: dashboardState.isRedAlliance ? const Color(0xffff4b6f) : const Color(0xff5d8cff)),
            const SizedBox(width: 10),
            Expanded(
              child: _StatusPill(
                label: dashboardState.robotEnabled ? dashboardState.robotMode : 'Disabled',
                color: dashboardState.robotEnabled ? const Color(0xff46d37d) : const Color(0xff737b94),
              ),
            ),
          ],
        ),
        const SizedBox(height: 10),
        _StatusPill(
          label: _autoDriveStatus(),
          color: dashboardState.autoDriveActive
              ? const Color(0xff46d37d)
              : dashboardState.lastRejectedReason.isEmpty
                  ? const Color(0xff737b94)
                  : const Color(0xffffc857),
        ),
        const SizedBox(height: 10),
        _AutoChooser(dashboardState: dashboardState),
      ],
    );
  }

  String _autoDriveStatus() {
    if (dashboardState.autoDriveActive) {
      return 'AutoDrive Running';
    }
    if (dashboardState.lastRejectedReason.isNotEmpty) {
      return 'AutoDrive: ${dashboardState.lastRejectedReason}';
    }
    if (dashboardState.lastAcceptedRequestId >= 0) {
      return 'AutoDrive Ready';
    }
    return 'AutoDrive Waiting';
  }
}

class _AutoChooser extends StatelessWidget {
  const _AutoChooser({required this.dashboardState});

  final DashboardState dashboardState;

  @override
  Widget build(BuildContext context) {
    final options = dashboardState.autoOptions;
    final selected = options.contains(dashboardState.selectedAutoName)
        ? dashboardState.selectedAutoName
        : null;

    return Container(
      height: 46,
      padding: const EdgeInsets.symmetric(horizontal: 12),
      decoration: BoxDecoration(
        color: const Color(0xff171c31),
        border: Border.all(color: const Color(0xff303957)),
        borderRadius: BorderRadius.circular(8),
      ),
      child: Row(
        children: [
          const Text(
            'Auto',
            style: TextStyle(
              color: Color(0xffcbd4ff),
              fontWeight: FontWeight.w800,
            ),
          ),
          const SizedBox(width: 12),
          Expanded(
            child: DropdownButtonHideUnderline(
              child: DropdownButton<String>(
                value: selected,
                isExpanded: true,
                hint: const Text('Waiting for options'),
                borderRadius: BorderRadius.circular(8),
                items: options
                    .map(
                      (option) => DropdownMenuItem<String>(
                        value: option,
                        child: Text(option, overflow: TextOverflow.ellipsis),
                      ),
                    )
                    .toList(),
                onChanged:
                    options.isEmpty ? null : (value) => _selectAuto(value),
              ),
            ),
          ),
        ],
      ),
    );
  }

  void _selectAuto(String? value) {
    if (value == null) {
      return;
    }

    dashboardState.selectAuto(value);
  }
}

class _StatusPill extends StatelessWidget {
  const _StatusPill({required this.label, required this.color});

  final String label;
  final Color color;

  @override
  Widget build(BuildContext context) {
    return Container(
      height: 38,
      padding: const EdgeInsets.symmetric(horizontal: 14),
      alignment: Alignment.center,
      decoration: BoxDecoration(
        color: color.withValues(alpha: 0.18),
        border: Border.all(color: color.withValues(alpha: 0.7)),
        borderRadius: BorderRadius.circular(8),
      ),
      child: Text(
        label,
        overflow: TextOverflow.ellipsis,
        style: TextStyle(color: color, fontWeight: FontWeight.w700),
      ),
    );
  }
}
