import 'package:dashboard24/services/dashboard_state.dart';
import 'package:flutter/material.dart';

class ChuteSelector extends StatefulWidget {
  final DashboardState dashboardState;
  final bool redAlliance;

  const ChuteSelector({
    super.key,
    required this.dashboardState,
    required this.redAlliance,
  });

  @override
  State<ChuteSelector> createState() => _ChuteSelectorState();
}

class _ChuteSelectorState extends State<ChuteSelector> {
  int _selected = 0;

  @override
  void initState() {
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    Color activeColor = widget.redAlliance ? Colors.red[700]! : Colors.indigo;

    return SizedBox(
      width: 350,
      height: 400,
      child: Stack(
        children: [
          _buildButton(120, 170, 0, activeColor),
          _buildButton(150, 120, 1, activeColor),
          _buildButton(180, 70, 2, activeColor),
          _buildButton(210, 20, 3, activeColor),
        ],
      ),
    );
  }

  Widget _buildButton(double leftOffset, double topOffset, int level, Color activeColor) {
    return Positioned(
      left: widget.redAlliance ? leftOffset : null,
      right: widget.redAlliance ? null : leftOffset,
      top: topOffset,
      child: Transform.scale(
        scale: 2.0,
        child: ElevatedButton(
          style: ElevatedButton.styleFrom(
            backgroundColor: _selected == level ? activeColor : Colors.grey,
            shape: const CircleBorder(),
            padding: const EdgeInsets.all(10),
          ),
          onPressed: () {
            setState(() {
              _selected = level;
              widget.dashboardState.setChuteMode(_selected);
            });
          },
          child: Text('${level + 1}', style: const TextStyle(color: Colors.white, fontSize: 14)),
        ),
      ),
    );
  }
}
