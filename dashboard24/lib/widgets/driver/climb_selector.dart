import 'package:dashboard24/services/dashboard_state.dart';
import 'package:flutter/material.dart';

class ClimbSelector extends StatefulWidget { //The good one, the reef
  final DashboardState dashboardState;
  final bool redAlliance;

  const ClimbSelector({
    super.key,
    required this.dashboardState,
    required this.redAlliance,
  });

  @override
  State<ClimbSelector> createState() => _ClimbSelectorState();
}

class _ClimbSelectorState extends State<ClimbSelector> {
  int _selected = 1;

  @override
  Widget build(BuildContext context) {
    Color activeColor = widget.redAlliance ? Colors.red[700]! : Colors.indigo;

    return SizedBox(
      width: 600,
      height: 600,
      child: Padding(
        padding: const EdgeInsets.all(8.0),
        child: Stack(
          alignment: Alignment.center,
          children: [
            Transform.scale(
              scale: 1.3,
              child: Image.asset('images/reef.png'),
            ),
            Positioned(
              left: 100,
              top: 70,
              child: _buildCheckbox(0, activeColor),
            ),
            Positioned(
              left: 190,
              top: 30,
              child: _buildCheckbox(1, activeColor),
            ),
            Positioned(
              left: 370,
              top: 20,
              child: _buildCheckbox(2, activeColor),
            ),
            Positioned(
              left: 460,
              top: 70,
              child: _buildCheckbox(3, activeColor),
            ),
            Positioned(
              left: 550,
              top: 220,
              child: _buildCheckbox(4, activeColor),
            ),
            Positioned(
              left: 550,
              top: 320,
              child: _buildCheckbox(5, activeColor),
            ),
            Positioned(
              left: 360,
              top: 520,
              child: _buildCheckbox(7, activeColor),
            ),
            Positioned(
              left: 460,
              top: 470,
              child: _buildCheckbox(6, activeColor),
            ),
            Positioned(
              left: 180,
              top: 520,
              child: _buildCheckbox(8, activeColor),
            ),
            Positioned(
              left: 90,
              top: 480,
              child: _buildCheckbox(9, activeColor),
            ),
            Positioned(
              left: 0,
              top: 240,
              child: _buildCheckbox(11, activeColor),
            ),
            Positioned(
              left: 0,
              top: 330,
              child: _buildCheckbox(10, activeColor),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildCheckbox(int position, Color activeColor) {
    return Transform.scale(
      scale: 4.0,
      child: Checkbox(
        value: _selected == position,
        splashRadius: 10.0,
        checkColor: Colors.white,
        activeColor: activeColor,
        shape: const CircleBorder(),
        side: const BorderSide(width: 0.5, color: Colors.grey),
        onChanged: (value) {
          setState(() {
            if (value ?? false) {
              _selected = position;
              widget.dashboardState.setClimbPos(_selected);
            }
          });
        },
      ),
    );
  }
}