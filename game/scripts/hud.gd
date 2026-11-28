class_name Hud
extends CanvasLayer
## Debug readout of the boat's state, updated at a fixed low rate, with a wind-direction dial. The
## toggle_hud action shows and hides it.

const REFRESH_INTERVAL_S := 0.1
const DIAL_RADIUS := 48.0
const MODE_NAMES: Array[String] = ["chase", "orbit", "helm"]

@export var boat_path: NodePath
@export var camera_rig_path: NodePath

var readout: Label
var dial: WindDial

var _boat: Node
var _rig: Node
var _since_refresh := 0.0


## Draws the heading-up dial. Angles are degrees from the bow, positive towards starboard.
class WindDial extends Control:
	const TRUE_COLOR := Color(0.3, 0.8, 1.0)
	const APPARENT_COLOR := Color(1.0, 0.8, 0.2)

	var true_wind_deg := 0.0
	var apparent_wind_deg := 0.0

	func _draw() -> void:
		var centre := size * 0.5
		draw_arc(centre, DIAL_RADIUS, 0.0, TAU, 48, Color(1, 1, 1, 0.5), 1.5)
		draw_line(centre, centre + Vector2(0.0, -DIAL_RADIUS), Color(1, 1, 1, 0.8), 2.0)
		_arrow(centre, true_wind_deg, TRUE_COLOR, DIAL_RADIUS)
		_arrow(centre, apparent_wind_deg, APPARENT_COLOR, DIAL_RADIUS * 0.7)

	# The tail sits on the side the wind comes from and the head points at the centre.
	func _arrow(centre: Vector2, from_deg: float, color: Color, length: float) -> void:
		var out := Vector2(sin(deg_to_rad(from_deg)), -cos(deg_to_rad(from_deg)))
		var tail := centre + out * length
		var head := centre + out * 4.0
		draw_line(tail, head, color, 2.5)
		var side := out.orthogonal() * 5.0
		draw_colored_polygon(PackedVector2Array([head, head + out * 10.0 + side, head + out * 10.0 - side]), color)

	func set_angles(true_deg: float, apparent_deg: float) -> void:
		true_wind_deg = true_deg
		apparent_wind_deg = apparent_deg
		queue_redraw()


func _ready() -> void:
	_boat = get_node_or_null(boat_path)
	_rig = get_node_or_null(camera_rig_path)

	var root := MarginContainer.new()
	root.mouse_filter = Control.MOUSE_FILTER_IGNORE
	root.add_theme_constant_override("margin_left", 8)
	root.add_theme_constant_override("margin_top", 8)
	add_child(root)

	var row := HBoxContainer.new()
	row.mouse_filter = Control.MOUSE_FILTER_IGNORE
	root.add_child(row)

	readout = Label.new()
	var font := SystemFont.new()
	font.font_names = PackedStringArray(["monospace"])
	readout.add_theme_font_override("font", font)
	readout.add_theme_font_size_override("font_size", 14)
	readout.add_theme_color_override("font_outline_color", Color.BLACK)
	readout.add_theme_constant_override("outline_size", 4)
	readout.mouse_filter = Control.MOUSE_FILTER_IGNORE
	row.add_child(readout)

	dial = WindDial.new()
	dial.custom_minimum_size = Vector2.ONE * (DIAL_RADIUS * 2.0 + 8.0)
	dial.mouse_filter = Control.MOUSE_FILTER_IGNORE
	row.add_child(dial)

	refresh()


func _process(delta: float) -> void:
	if not visible:
		return
	_since_refresh += delta
	if _since_refresh >= REFRESH_INTERVAL_S:
		refresh()


func _unhandled_input(event: InputEvent) -> void:
	if event.is_action_pressed(&"toggle_hud"):
		visible = not visible
		get_viewport().set_input_as_handled()


func refresh() -> void:
	_since_refresh = 0.0
	if _boat == null:
		readout.text = "no boat"
		return
	var s: Dictionary = _boat.get_state()
	if not s.get("active", false):
		readout.text = "boat inactive"
		return
	var lines: Array[String] = []
	lines.append("BSP %5.2f kn   heading %5.1f" % [s["bsp_kn"], s["heading_deg"]])
	lines.append("TWS %5.1f kn   TWD %5.1f   TWA %6.1f (course %6.1f)" % [s["tws_kn"], s["twd_deg"], s["twa_deg"], s["course_twa_deg"]])
	lines.append("AWS %5.1f kn   AWA %6.1f" % [s["aws_kn"], s["awa_deg"]])
	lines.append("heel %5.1f   leeway %5.1f   rudder %5.1f deg (hinge %.0f Nm)" % [s["heel_deg"], s["leeway_deg"], s["rudder_angle_deg"], s["rudder_hinge_moment_nm"]])
	lines.append("tiller %5.2f  main %4.2f  traveler %5.2f  backstay %4.2f" % [_boat.tiller, _boat.mainsheet, _boat.traveler, _boat.backstay])
	lines.append("jib sheet port %4.2f stbd %4.2f  car %4.2f  reef %d" % [_boat.jib_sheet_port, _boat.jib_sheet_starboard, _boat.jib_car, int(_boat.reef)])
	lines.append("main %s   jib %s" % [_sail_flags(s, "main"), _sail_flags(s, "jib")])
	lines.append("crew %5.2f m to port, %4.2f m high%s" % [s["crew_to_port_m"], s["crew_height_m"], " (hiking)" if s["crew_hiking"] else ""])
	if int(s.get("seabed_contacts", 0)) > 0:
		lines.append("AGROUND: %d contacts, %.0f N" % [s["seabed_contacts"], s["seabed_normal_force_n"]])
	lines.append("camera %s" % _camera_mode())
	lines.append("step %6.1f us" % _boat.get_step_time_us())
	readout.text = "\n".join(lines)
	dial.set_angles(float(s["twa_deg"]), float(s["awa_deg"]))


func _sail_flags(s: Dictionary, sail: String) -> String:
	var flags: Array[String] = []
	if s.get(sail + "_luffing", false):
		flags.append("LUFF")
	if s.get(sail + "_stalled", false):
		flags.append("STALL")
	if s.get(sail + "_backed", false):
		flags.append("BACKED")
	return "ok" if flags.is_empty() else " ".join(flags)


func _camera_mode() -> String:
	if _rig == null:
		return "-"
	return MODE_NAMES[int(_rig.mode)]
