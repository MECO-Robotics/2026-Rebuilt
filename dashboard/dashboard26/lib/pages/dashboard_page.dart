import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

import '../models/field_target.dart';
import '../services/dashboard_state.dart';
import '../widgets/auto_drive_field.dart';
import '../widgets/game_piece_panel.dart';
import '../widgets/status_panel.dart';

class DashboardPage extends StatefulWidget {
  const DashboardPage({required this.dashboardState, super.key});

  final DashboardState dashboardState;

  @override
  State<DashboardPage> createState() => _DashboardPageState();
}

class _DashboardPageState extends State<DashboardPage> {
  final FocusNode keyboardFocusNode = FocusNode();
  TargetGroup selectedGroup = TargetGroup.hub;

  @override
  void dispose() {
    keyboardFocusNode.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: widget.dashboardState,
      builder: (context, _) {
        final visibleTargets = autoDriveTargets
            .where((target) => target.group == selectedGroup)
            .toList();
        return KeyboardListener(
          focusNode: keyboardFocusNode,
          autofocus: true,
          onKeyEvent: (event) {
            if (event is! KeyDownEvent) {
              return;
            }
            if (event.logicalKey == LogicalKeyboardKey.enter ||
                event.logicalKey == LogicalKeyboardKey.numpadEnter) {
              widget.dashboardState.killRobot();
            }
          },
          child: Scaffold(
            backgroundColor: const Color(0xff111525),
            body: SafeArea(
              child: Padding(
                padding: const EdgeInsets.all(18),
                child: Row(
                  children: [
                    Expanded(
                      flex: 7,
                      child: AutoDriveField(
                        targets: visibleTargets,
                        redAlliance: widget.dashboardState.isRedAlliance,
                        onPoseSelected: widget.dashboardState.driveToPose,
                      ),
                    ),
                    const SizedBox(width: 18),
                    SizedBox(
                      width: 460,
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.stretch,
                        children: [
                          StatusPanel(dashboardState: widget.dashboardState),
                          const SizedBox(height: 18),
                          GamePiecePanel(dashboardState: widget.dashboardState),
                          const SizedBox(height: 18),
                          _GroupSelector(
                            selectedGroup: selectedGroup,
                            onChanged: (group) =>
                                setState(() => selectedGroup = group),
                          ),
                          const SizedBox(height: 18),
                          Expanded(
                            child: _TargetList(
                              targets: visibleTargets,
                              onSelected: (target) =>
                                  widget.dashboardState.driveToPose(
                                target.xMeters,
                                target.yMeters,
                                target.headingDegrees,
                              ),
                            ),
                          ),
                        ],
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ),
        );
      },
    );
  }
}

class _GroupSelector extends StatelessWidget {
  const _GroupSelector({required this.selectedGroup, required this.onChanged});

  final TargetGroup selectedGroup;
  final ValueChanged<TargetGroup> onChanged;

  @override
  Widget build(BuildContext context) {
    return Wrap(
      spacing: 8,
      runSpacing: 8,
      children: TargetGroup.values.map((group) {
        final selected = selectedGroup == group;
        return ChoiceChip(
          label: Text(_labelFor(group)),
          selected: selected,
          onSelected: (_) => onChanged(group),
          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
        );
      }).toList(),
    );
  }

  String _labelFor(TargetGroup group) {
    return switch (group) {
      TargetGroup.hub => 'Hub',
      TargetGroup.trench => 'Trench',
      TargetGroup.depot => 'Depot',
      TargetGroup.outpost => 'Outpost',
      TargetGroup.climb => 'Climb',
      TargetGroup.bump => 'Bump',
      TargetGroup.counter => 'Counter',
    };
  }
}

class _TargetList extends StatelessWidget {
  const _TargetList({required this.targets, required this.onSelected});

  final List<FieldTarget> targets;
  final ValueChanged<FieldTarget> onSelected;

  @override
  Widget build(BuildContext context) {
    return ListView.separated(
      itemCount: targets.length,
      separatorBuilder: (_, __) => const SizedBox(height: 10),
      itemBuilder: (context, index) {
        final target = targets[index];
        return FilledButton(
          style: FilledButton.styleFrom(
            alignment: Alignment.centerLeft,
            minimumSize: const Size.fromHeight(56),
            shape:
                RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
          ),
          onPressed: () => onSelected(target),
          child: Text(target.name, overflow: TextOverflow.ellipsis),
        );
      },
    );
  }
}
