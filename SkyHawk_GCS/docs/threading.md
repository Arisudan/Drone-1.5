# Drone GCS threading contract

One rule: **workers never touch widgets.** Every worker hands data to the UI
thread through a Qt signal (queued connection), and the UI thread owns all state
that widgets read. Slots must be quick; anything slow goes to a worker.

| Thread | Class | Signals to the UI thread (payload) |
|---|---|---|
| MAVLink I/O + reconnect loop | `MAVLinkWorker` | `telemetry_updated(TelemetrySnapshot)`, `connection_changed(bool,str)`, `statustext_received(str,int)`, `command_ack_received(int,int,str,str)`, `rates_updated(float,float)`, `command_broadcast_received(str)`, `param_value_received(...)`, `motor_param_received(str,float)` (only the `PWM_MAIN_{MIN,MAX,DIS,FUNC}n` parameters, for the motor scale; a lone answer to a single read is not forwarded to `param_value_received`, so the Parameters tab is not tricked into thinking it has data) |
| Path planning / collision / quality / inflation | `PlannerWorker` | `plan_ready(token,plan)`, `collision_ready(token,ok,pt,dist)`, `quality_ready(token,q)`, `inflation_ready(token,img,meta)`, `job_timing(str,float)` |
| ROS 2 map subscription + TCP fallback | `ROS2MapListener` (+ inner TCP `threading.Thread`) | `map_received(grid,res,ox,oy,src,extra)`, `status_updated(str)` |
| Map reset | `SlamMapResetWorker` | `finished_result(bool,str)` |
| Video capture | `VideoCaptureThread` | `frame_ready(ndarray)` |
| Audio playback | `core/audio.py` (plain thread) | none - fire and forget |
| Health watchdog | `core/health.py` (plain thread) | none - read via polled snapshot |
| Actuator HTTP | `QNetworkAccessManager` (UI thread, async) | reply slots run on the UI thread |

## Rules

1. **Tokens, not flags.** Planner results carry the job token; a stale token is
   discarded in the slot (a verdict about a replaced path must never halt the
   current one).
2. **Telemetry is a snapshot.** `telemetry_updated` delivers an object the UI
   thread may read freely; the worker builds a new one rather than mutating the
   one it already emitted.
3. **Shutdown joins workers.** `shutdown_workers()` stops and waits on every
   QThread; a QThread destroyed while running aborts the process.
4. **Stall detector.** The 30 Hz UI tick feeds `core/ui_stall.py`; a gap over
   250 ms logs a console warning (rate limited). If it fires, something
   blocked the UI thread - find the slot that did file/network/heavy numpy work.
