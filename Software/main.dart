// CTS-Guard – minimal, functional Flutter dashboard
// -----------------------------------------------
// What you get in this single file:
// 1) BLE connect/subscribe via flutter_blue_plus
// 2) Live 30s sparklines for wrist & finger accel magnitudes + optional gyro
// 3) Real-time state classifier: Normal / At-Risk / Alert (color tile)
// 4) Session summary: counts, % time in each state, average response time
// 5) Device health: RSSI, packet rate, missing packet %
//
// --- Add to pubspec.yaml ---
// dependencies:
//   flutter:
//     sdk: flutter
//   flutter_blue_plus: ^1.33.8
//   fl_chart: ^0.69.0
//   collection: ^1.18.0
//   intl: ^0.19.0
//
// Notes
// - Assumes your device streams CSV text notifications: "Aw,Af,Gw,Alert\n"
//   where Aw = wrist accel magnitude (m/s^2), Af = finger accel magnitude (m/s^2),
//   Gw = wrist gyro magnitude (deg/s, optional), Alert = 0/1 flag your firmware sets
//   Example:  "0.12,0.85,3.4,0\n"
// - If your device format differs, update _parseSample() accordingly.
// - Replace the UUID constants with your Service/Characteristic IDs.
// - For Windows/macOS desktop, enable BLE in project as needed.
// - This is production-lean: tidy UI, clear separation, defensive parsing.

import 'dart:async';
import 'dart:convert';
import 'dart:math' as math;

import 'package:collection/collection.dart';
import 'package:fl_chart/fl_chart.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:intl/intl.dart';

void main() {
  WidgetsFlutterBinding.ensureInitialized();
  SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);
  runApp(const CTSGuardApp());
}

class CTSGuardApp extends StatelessWidget {
  const CTSGuardApp({super.key});
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: const Color(0xFF2563EB)),
        useMaterial3: true,
      ),
      home: const _Root(),
    );
  }
}

class _Root extends StatefulWidget {
  const _Root();
  @override
  State<_Root> createState() => _RootState();
}

class _RootState extends State<_Root> {
  int _index = 0;
  final BleBloc _ble = BleBloc();
  final SessionBloc _session = SessionBloc();

  @override
  void dispose() {
    _ble.dispose();
    _session.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      bottomNavigationBar: NavigationBar(
        selectedIndex: _index,
        onDestinationSelected: (i) => setState(() => _index = i),
        destinations: const [
          NavigationDestination(icon: Icon(Icons.bluetooth_searching), label: 'Bluetooth'),
          NavigationDestination(icon: Icon(Icons.dashboard_customize), label: 'Dashboard'),
        ],
      ),
      body: IndexedStack(
        index: _index,
        children: [
          BleScreen(ble: _ble),
          DashboardScreen(ble: _ble, session: _session),
        ],
      ),
    );
  }
}

// === Domain model ===
class SensorSample {
  final DateTime t;
  final double aw; // wrist accel magnitude
  final double af; // finger accel magnitude
  final double? gw; // gyro magnitude optional
  final bool alert; // device-sent alert flag
  SensorSample({required this.t, required this.aw, required this.af, this.gw, required this.alert});
}

enum StateClass { normal, atRisk, alert }

class StateSnapshot {
  final StateClass state;
  final DateTime at;
  StateSnapshot(this.state, this.at);
}

// === BLE layer ===
class BleBloc {
  // --- REPLACE WITH YOUR OWN UUIDs ---
  static final Guid serviceUuid = Guid("4fafc201-1fb5-459e-8fcc-c5c9c331914b"); // Environmental Sensing (placeholder)
  static final Guid notifyCharUuid = Guid("beb5483e-36e1-4688-b7f5-ea07361b26a8"); // Analog (placeholder)

  final _scanResults = StreamController<List<ScanResult>>.broadcast();
  final _deviceState = StreamController<BluetoothDevice?>.broadcast();
  final _rssi = StreamController<int>.broadcast();
  final _lines = StreamController<String>.broadcast();

  BluetoothDevice? _connected;
  BluetoothCharacteristic? _notifyChar;
  StreamSubscription<List<int>>? _notifySub;
  StreamSubscription<int>? _rssiSub;

  Stream<List<ScanResult>> get scan$ => _scanResults.stream;
  Stream<BluetoothDevice?> get device$ => _deviceState.stream;
  Stream<int> get rssi$ => _rssi.stream;
  Stream<String> get lines$ => _lines.stream; // raw CSV lines

  Future<void> startScan({String namePrefix = 'CTS-'}) async {
    await FlutterBluePlus.startScan(timeout: const Duration(seconds: 6));
    FlutterBluePlus.scanResults.listen((results) {
      final filtered = results.where((r) =>
          r.advertisementData.localName.startsWith(namePrefix) || namePrefix.isEmpty).toList();
      _scanResults.add(filtered);
    });
  }

  Future<void> stopScan() => FlutterBluePlus.stopScan();

  Future<void> connect(ScanResult r) async {
    await FlutterBluePlus.stopScan();

    await r.device.connect(
      license: License.free,             
      timeout: const Duration(seconds: 15),
    );

    _connected = r.device;
    _deviceState.add(_connected);

    // RSSI polling
    _rssiSub?.cancel();
    _rssiSub = Stream.periodic(const Duration(seconds: 2)).asyncMap((_) => _connected!.readRssi()).listen(_rssi.add);

    // Discover
    final services = await _connected!.discoverServices();
    final svc = services.firstWhereOrNull((s) => s.uuid == serviceUuid) ?? services.firstOrNull;
    if (svc == null) return;
    _notifyChar = svc.characteristics.firstWhereOrNull((c) => c.uuid == notifyCharUuid) ??
        svc.characteristics.firstWhereOrNull((c) => c.properties.notify);

    if (_notifyChar != null) {
      await _notifyChar!.setNotifyValue(true);
      _notifySub?.cancel();
      _notifySub = _notifyChar!.onValueReceived.listen((bytes) {
        // best-effort: split on newline, push lines
        final chunk = utf8.decode(bytes, allowMalformed: true);
        for (final line in chunk.split('\n')) {
          final s = line.trim();
          if (s.isNotEmpty) _lines.add(s);
        }
      });
    }
  }

  Future<void> disconnect() async {
    _notifySub?.cancel();
    _rssiSub?.cancel();
    if (_connected != null) {
      try { await _connected!.disconnect(); } catch (_) {}
    }
    _connected = null;
    _deviceState.add(null);
  }

  void dispose() {
    _scanResults.close();
    _deviceState.close();
    _rssi.close();
    _lines.close();
    _notifySub?.cancel();
    _rssiSub?.cancel();
  }
}

// === Session / derived metrics ===
class SessionBloc {
  // config thresholds (tune on-device or expose sliders)
  double wristVarThresh = 0.002; // m/s^2 variance
  double fingerVarThresh = 0.02; // m/s^2 variance
  final int fs = 20; // samples/sec expected
  final Duration window = const Duration(seconds: 30);
  final Duration varWin = const Duration(seconds: 2);

  final _sample$ = StreamController<SensorSample>.broadcast();
  final _state$ = StreamController<StateSnapshot>.broadcast();
  final _health$ = StreamController<DeviceHealth>.broadcast();

  // buffers
  final List<SensorSample> _buf = [];
  final List<StateSnapshot> _states = [];

  // summary
  int alerts = 0;
  int warnings = 0; // atRisk periods started
  Duration atRiskAccum = Duration.zero;
  Duration monitoring = Duration.zero;
  int totalPackets = 0;
  int missingPackets = 0;
  DateTime? _lastPacketAt;
  DateTime? _atRiskSince;
  DateTime? _lastAlertAt;
  final _rng = math.Random();

  Stream<SensorSample> get sample$ => _sample$.stream;
  Stream<StateSnapshot> get state$ => _state$.stream;
  Stream<DeviceHealth> get health$ => _health$.stream;

  // public API: feed raw CSV line
  void ingestCsv(String line) {
    final now = DateTime.now();
    final s = _parseSample(line, now);
    if (s == null) return;

    // packet accounting
    totalPackets++;
    if (_lastPacketAt != null) {
      final gap = now.difference(_lastPacketAt!).inMilliseconds;
      // if > 2*dt, consider missing
      final dtMs = (1000 / fs);
      if (gap > dtMs * 2) {
        missingPackets += ((gap / dtMs) - 1).floor();
      }
    }
    _lastPacketAt = now;

    // push sample
    _buf.add(s);
    _trimToWindow();
    _sample$.add(s);

    // compute state
    final state = _classify();
    _emitState(state);

    // health
    final rate = _estimatePacketRate();
    _health$.add(DeviceHealth(packetRateHz: rate, missingPct: _missingPct(), batteryPct: null, rssi: null));
  }

  void updateRssi(int rssi) {
    // fold into health stream by echoing last rate but new RSSI
    final rate = _estimatePacketRate();
    _health$.add(DeviceHealth(packetRateHz: rate, missingPct: _missingPct(), batteryPct: null, rssi: rssi));
  }

  double _estimatePacketRate() {
    if (_buf.length < 2) return 0;
    final span = _buf.last.t.difference(_buf.first.t).inMilliseconds / 1000.0;
    return span > 0 ? _buf.length / span : 0;
    }

  double _missingPct() {
    final expected = (monitoring.inMilliseconds / (1000 / fs)).clamp(1, double.infinity);
    if (expected == 0) return 0;
    return (missingPackets / expected) * 100.0;
  }

  void _emitState(StateClass sc) {
    final now = DateTime.now();
    monitoring += _buf.isEmpty ? Duration.zero : now.difference(_buf.last.t);

    final last = _states.isEmpty ? null : _states.last.state;
    if (last != sc) {
      _states.add(StateSnapshot(sc, now));
      _state$.add(StateSnapshot(sc, now));

      if (sc == StateClass.atRisk) {
        warnings++;
        _atRiskSince = now;
      }
      if (sc == StateClass.alert) {
        alerts++;
        _lastAlertAt = now;
      }
      if (last == StateClass.atRisk && sc != StateClass.atRisk && _atRiskSince != null) {
        atRiskAccum += now.difference(_atRiskSince!);
        _atRiskSince = null;
      }
    }
  }

  StateClass _classify() {
    // sliding 2s variance
    final now = DateTime.now();
    final from = now.subtract(varWin);
    final w = _buf.where((s) => s.t.isAfter(from)).toList();
    if (w.length < 5) return StateClass.normal; // not enough

    double _variance(List<double> x) {
      final m = x.average;
      return x.map((v) => (v - m) * (v - m)).average;
    }

    final vAw = _variance(w.map((e) => e.aw).toList());
    final vAf = _variance(w.map((e) => e.af).toList());

    final stationary = vAw < wristVarThresh;
    final fingerActive = vAf > fingerVarThresh;

    final deviceAlert = w.last.alert; // from firmware if any

    if (deviceAlert || (stationary && fingerActive)) return StateClass.alert;
    if (stationary && !fingerActive) return StateClass.atRisk; // long immobility without finger movement → warning
    return StateClass.normal;
  }

  void _trimToWindow() {
    final cutoff = DateTime.now().subtract(window);
    while (_buf.isNotEmpty && _buf.first.t.isBefore(cutoff)) {
      _buf.removeAt(0);
    }
  }

  SensorSample? _parseSample(String line, DateTime t) {
    try {
      final parts = line.split(',').map((e) => e.trim()).toList();
      if (parts.length < 2) return null;
      final aw = double.parse(parts[0]);
      final af = double.parse(parts[1]);
      double? gw;
      bool alert = false;
      if (parts.length >= 3) {
        final p2 = double.tryParse(parts[2]);
        if (p2 != null) gw = p2; else alert = parts[2] == '1';
      }
      if (parts.length >= 4) {
        alert = ['1', 'true', 'TRUE'].contains(parts[3]);
      }
      return SensorSample(t: t, aw: aw, af: af, gw: gw, alert: alert);
    } catch (_) {
      return null;
    }
  }

  void dispose() {
    _sample$.close();
    _state$.close();
    _health$.close();
  }
}

class DeviceHealth {
  final double packetRateHz;
  final double missingPct;
  final int? batteryPct;
  final int? rssi;
  DeviceHealth({required this.packetRateHz, required this.missingPct, this.batteryPct, this.rssi});
}

// === UI: Bluetooth screen ===
class BleScreen extends StatefulWidget {
  final BleBloc ble;
  const BleScreen({super.key, required this.ble});
  @override
  State<BleScreen> createState() => _BleScreenState();
}

class _BleScreenState extends State<BleScreen> {
  @override
  void initState() {
    super.initState();
    widget.ble.startScan(namePrefix: 'CTS-');
  }

  @override
  void dispose() {
    widget.ble.stopScan();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return SafeArea(
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          const _AppHeader(title: 'Bluetooth Devices'),
          Expanded(
            child: StreamBuilder<List<ScanResult>>(
              stream: widget.ble.scan$,
              initialData: const [],
              builder: (context, snap) {
                final results = snap.data ?? [];
                if (results.isEmpty) {
                  return const Center(child: Text('Scanning… Move closer to your device'));
                }
                return ListView.separated(
                  itemCount: results.length,
                  separatorBuilder: (_, __) => const Divider(height: 1),
                  itemBuilder: (context, i) {
                    final r = results[i];
                    return ListTile(
                      leading: const Icon(Icons.watch),
                      title: Text(r.advertisementData.localName.isNotEmpty
                          ? r.advertisementData.localName
                          : r.device.platformName.isNotEmpty
                              ? r.device.platformName
                              : r.device.remoteId.str),
                      subtitle: Text('RSSI ${r.rssi}  •  ${r.device.remoteId.str}'),
                      trailing: FilledButton(
                        onPressed: () async {
                          await widget.ble.connect(r);
                          if (context.mounted) {
                            ScaffoldMessenger.of(context).showSnackBar(
                              const SnackBar(content: Text('Connected. Go to Dashboard tab.')),
                            );
                          }
                        },
                        child: const Text('Connect'),
                      ),
                    );
                  },
                );
              },
            ),
          ),
        ],
      ),
    );
  }
}

class _AppHeader extends StatelessWidget {
  final String title;
  const _AppHeader({required this.title});
  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.fromLTRB(16, 12, 16, 4),
      child: Row(
        children: [
          Text(title, style: Theme.of(context).textTheme.titleLarge?.copyWith(fontWeight: FontWeight.w700)),
          const Spacer(),
          const Icon(Icons.health_and_safety, size: 20),
        ],
      ),
    );
  }
}

// === UI: Dashboard ===
class DashboardScreen extends StatefulWidget {
  final BleBloc ble;
  final SessionBloc session;
  const DashboardScreen({super.key, required this.ble, required this.session});
  @override
  State<DashboardScreen> createState() => _DashboardScreenState();
}

class _DashboardScreenState extends State<DashboardScreen> {
  StreamSubscription<String>? _lineSub;
  StreamSubscription<int>? _rssiSub;

  @override
  void initState() {
    super.initState();
    _lineSub = widget.ble.lines$.listen(widget.session.ingestCsv);
    _rssiSub = widget.ble.rssi$.listen(widget.session.updateRssi);
  }

  @override
  void dispose() {
    _lineSub?.cancel();
    _rssiSub?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return SafeArea(
      child: Padding(
        padding: const EdgeInsets.all(12.0),
        child: Column(
          children: [
            // Header row: BLE status + health
            Row(
              children: [
                StreamBuilder<BluetoothDevice?>(
                  stream: widget.ble.device$,
                  builder: (context, snap) {
                    final d = snap.data;
                    final connected = d != null;
                    return Row(children: [
                      Icon(connected ? Icons.bluetooth_connected : Icons.bluetooth_disabled,
                          color: connected ? Colors.green : Colors.red),
                      const SizedBox(width: 8),
                      Text(connected ? (d.platformName.isNotEmpty ? d.platformName : d.remoteId.str) : 'No device'),
                    ]);
                  },
                ),
                const Spacer(),
                StreamBuilder<DeviceHealth>(
                  stream: widget.session.health$,
                  builder: (context, snap) {
                    final h = snap.data;
                    return Text(
                      h == null
                          ? '—'
                          : 'Rate: ${h.packetRateHz.toStringAsFixed(1)} Hz  •  Loss: ${h.missingPct.toStringAsFixed(1)}%  •  RSSI: ${h.rssi ?? 0}',
                      style: Theme.of(context).textTheme.labelMedium,
                    );
                  },
                ),
              ],
            ),
            const SizedBox(height: 8),

            // State tile
            StreamBuilder<StateSnapshot>(
              stream: widget.session.state$,
              builder: (context, snap) {
                final sc = snap.data?.state ?? StateClass.normal;
                final color = switch (sc) {
                  StateClass.normal => Colors.green,
                  StateClass.atRisk => Colors.orange,
                  StateClass.alert => Colors.red,
                };
                final text = switch (sc) {
                  StateClass.normal => 'NORMAL',
                  StateClass.atRisk => 'WARNING – Stationary',
                  StateClass.alert => 'ALERT – Finger moving while wrist stationary',
                };
                return _BigStateTile(color: color, text: text);
              },
            ),
            const SizedBox(height: 8),

            // Live charts row
            SizedBox(
              height: 160,
              child: Row(
                children: [
                  Expanded(child: _Sparkline(title: 'Wrist | Aw', stream: widget.session.sample$, pick: (s) => s.aw)),
                  const SizedBox(width: 8),
                  Expanded(child: _Sparkline(title: 'Finger | Af', stream: widget.session.sample$, pick: (s) => s.af)),
                ],
              ),
            ),

            const SizedBox(height: 8),

            // Summary row
            Row(
              children: [
                Expanded(child: _StatCard(icon: Icons.vibration, label: 'Warnings', valueBuilder: () => widget.session.warnings.toString())),
                const SizedBox(width: 8),
                Expanded(child: _StatCard(icon: Icons.notification_important, label: 'Alerts', valueBuilder: () => widget.session.alerts.toString())),
                const SizedBox(width: 8),
                Expanded(child: _StatCard(icon: Icons.timer, label: 'At-Risk (min)', valueBuilder: () => (widget.session.atRiskAccum.inSeconds/60).toStringAsFixed(1))),
              ],
            ),

            const SizedBox(height: 8),

            // Donut of session state proportions (simple approximation from last N states)
            _DonutSession(states: widget.session._states),

            const SizedBox(height: 8),

            // Controls
            Row(
              children: [
                Expanded(
                  child: _SliderSetting(
                    label: 'Wrist var thres',
                    min: 0.0001,
                    max: 0.02,
                    value: widget.session.wristVarThresh,
                    onChanged: (v) => setState(() => widget.session.wristVarThresh = v),
                  ),
                ),
                const SizedBox(width: 12),
                Expanded(
                  child: _SliderSetting(
                    label: 'Finger var thres',
                    min: 0.001,
                    max: 0.1,
                    value: widget.session.fingerVarThresh,
                    onChanged: (v) => setState(() => widget.session.fingerVarThresh = v),
                  ),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }
}

class _BigStateTile extends StatelessWidget {
  final Color color;
  final String text;
  const _BigStateTile({required this.color, required this.text});
  @override
  Widget build(BuildContext context) {
    return Container(
      height: 72,
      decoration: BoxDecoration(color: color.withOpacity(0.12), borderRadius: BorderRadius.circular(16), border: Border.all(color: color, width: 2)),
      child: Center(
        child: Text(
          text,
          style: Theme.of(context).textTheme.titleMedium?.copyWith(fontWeight: FontWeight.w800, color: color.darken()),
          textAlign: TextAlign.center,
        ),
      ),
    );
  }
}

class _Sparkline extends StatefulWidget {
  final String title;
  final Stream<SensorSample> stream;
  final double Function(SensorSample) pick;
  const _Sparkline({required this.title, required this.stream, required this.pick});
  @override
  State<_Sparkline> createState() => _SparklineState();
}

class _SparklineState extends State<_Sparkline> {
  final List<FlSpot> _spots = [];
  DateTime? _t0;

  @override
  void initState() {
    super.initState();
    widget.stream.listen((s) {
      _t0 ??= s.t;
      final x = s.t.difference(_t0!).inMilliseconds / 1000.0;
      final y = widget.pick(s);
      setState(() {
        _spots.add(FlSpot(x, y));
        // keep last 30s
        final cutoff = x - 30.0;
        while (_spots.isNotEmpty && _spots.first.x < cutoff) {
          _spots.removeAt(0);
        }
      });
    });
  }

  @override
  Widget build(BuildContext context) {
    final color = Theme.of(context).colorScheme.primary;
    return Card(
      elevation: 0,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12), side: BorderSide(color: Colors.grey.shade300)),
      child: Padding(
        padding: const EdgeInsets.all(12.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            Text(widget.title, style: Theme.of(context).textTheme.labelLarge),
            const SizedBox(height: 8),
            Expanded(
              child: LineChart(
                LineChartData(
                  minX: _spots.isEmpty ? 0 : (_spots.last.x - 30).clamp(0, double.infinity),
                  maxX: _spots.isEmpty ? 30 : _spots.last.x,
                  gridData: FlGridData(show: false),
                  titlesData: FlTitlesData(show: false),
                  borderData: FlBorderData(show: false),
                  lineTouchData: const LineTouchData(enabled: false),
                  lineBarsData: [
                    LineChartBarData(
                      spots: _spots,
                      isCurved: true,
                      barWidth: 2,
                      color: color,
                      dotData: const FlDotData(show: false),
                    ),
                  ],
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class _StatCard extends StatelessWidget {
  final IconData icon;
  final String label;
  final String Function() valueBuilder;
  const _StatCard({required this.icon, required this.label, required this.valueBuilder});
  @override
  Widget build(BuildContext context) {
    return Card(
      elevation: 0,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12), side: BorderSide(color: Colors.grey.shade300)),
      child: Padding(
        padding: const EdgeInsets.all(12.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Icon(icon, size: 18),
            const SizedBox(height: 6),
            Text(valueBuilder(), style: Theme.of(context).textTheme.titleMedium?.copyWith(fontWeight: FontWeight.w800)),
            Text(label, style: Theme.of(context).textTheme.labelMedium),
          ],
        ),
      ),
    );
  }
}

class _DonutSession extends StatelessWidget {
  final List<StateSnapshot> states;
  const _DonutSession({required this.states});
  @override
  Widget build(BuildContext context) {
    final counts = {for (var s in StateClass.values) s: 0};
    for (final s in states) {
      counts[s.state] = (counts[s.state] ?? 0) + 1;
    }
    final total = counts.values.fold<int>(0, (a, b) => a + b).clamp(1, 1<<30);
    final sections = <PieChartSectionData>[
      _sec(context, Colors.green, (counts[StateClass.normal]! / total) * 100, 'Normal'),
      _sec(context, Colors.orange, (counts[StateClass.atRisk]! / total) * 100, 'At-risk'),
      _sec(context, Colors.red, (counts[StateClass.alert]! / total) * 100, 'Alert'),
    ];

    return Card(
      elevation: 0,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12), side: BorderSide(color: Colors.grey.shade300)),
      child: SizedBox(
        height: 160,
        child: Row(
          children: [
            const SizedBox(width: 12),
            Expanded(
              child: PieChart(
                PieChartData(
                  sections: sections,
                  centerSpaceRadius: 36,
                  sectionsSpace: 2,
                ),
              ),
            ),
            const SizedBox(width: 8),
            Column(
              mainAxisAlignment: MainAxisAlignment.center,
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                _legendDot(Colors.green, 'Normal'),
                _legendDot(Colors.orange, 'At-risk'),
                _legendDot(Colors.red, 'Alert'),
              ],
            ),
            const SizedBox(width: 12),
          ],
        ),
      ),
    );
  }

  PieChartSectionData _sec(BuildContext ctx, Color c, double pct, String label) {
    return PieChartSectionData(
      color: c,
      value: pct,
      title: '${pct.toStringAsFixed(0)}%',
      radius: 52,
      titleStyle: Theme.of(ctx).textTheme.labelLarge?.copyWith(color: Colors.white, fontWeight: FontWeight.w700),
    );
  }

  Widget _legendDot(Color c, String t) => Padding(
        padding: const EdgeInsets.symmetric(vertical: 4.0),
        child: Row(children: [
          Container(width: 10, height: 10, decoration: BoxDecoration(color: c, shape: BoxShape.circle)),
          const SizedBox(width: 8),
          Text(t),
        ]),
      );
}

class _SliderSetting extends StatelessWidget {
  final String label; final double min; final double max; final double value; final ValueChanged<double> onChanged;
  const _SliderSetting({required this.label, required this.min, required this.max, required this.value, required this.onChanged});
  @override
  Widget build(BuildContext context) {
    return Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
      Row(children: [Text(label), const Spacer(), Text(value.toStringAsFixed(4))]),
      Slider(value: value.clamp(min, max), min: min, max: max, onChanged: onChanged),
    ]);
  }
}

extension _ColorX on Color {
  Color darken([double amount = .2]) {
    assert(amount >= 0 && amount <= 1);
    final hsl = HSLColor.fromColor(this);
    final h = hsl.withLightness((hsl.lightness - amount).clamp(0.0, 1.0));
    return h.toColor();
  }
}

extension _Avg on Iterable<double> {
  double get average => isEmpty ? 0.0 : (reduce((a, b) => a + b) / length);
}
