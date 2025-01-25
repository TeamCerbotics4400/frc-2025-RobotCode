import 'package:dashboard24/services/dashboard_state.dart';
import 'package:flutter/material.dart';

class ScoringMode extends StatefulWidget { //The level selector
  final DashboardState dashboardState;
  final bool redAlliance;

  const ScoringMode({
    super.key,
    required this.dashboardState,
    required this.redAlliance,
  });

  @override
  State<ScoringMode> createState() => _ScoringModeState();
}

class _ScoringModeState extends State<ScoringMode> {
  int _scoringMode = 1;

  @override
  void initState() {
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    Color activeColor = widget.redAlliance ? Colors.red[700]! : Colors.indigo;

    return SizedBox(
      width: 500,
      height: 400,
      child: Padding(
        padding: const EdgeInsets.all(8.0),
        child: Column(
          mainAxisSize: MainAxisSize.max,
          crossAxisAlignment: CrossAxisAlignment.center,
          mainAxisAlignment: MainAxisAlignment.spaceEvenly,
          children: [
            const Text(
              'Scoring Mode',
              style: TextStyle(fontSize: 48),
            ),
            const Divider(),
            Expanded(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                children: List.generate(4, (index) {
                  int level = 3 - index;
                  return Expanded(
                    child: ElevatedButton(
                      style: ElevatedButton.styleFrom(
                        minimumSize: const Size(double.infinity, 50),
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(12),
                        ),
                        backgroundColor: _scoringMode == level ? activeColor : null,
                      ),
                      child: Text(
                        'L${level + 1}',
                        style: const TextStyle(fontSize: 36, color: Colors.white),
                      ),
                      onPressed: () {
                        setState(() {
                          _scoringMode = level;
                          widget.dashboardState.setScoringMode(_scoringMode);
                        });
                      },
                    ),
                  );
                }),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
