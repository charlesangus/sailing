class_name BoatInput
extends Node
## Turns the input actions of project.godot into SailBoat controls. Bindings are in docs/dev.md.

signal camera_switch_requested
signal mark_drop_requested
signal settings_toggled

const SHEET_RATE := 0.25
const TRAVELER_RATE := 0.5
const HELM_RAMP_RATE := 2.0
const HELM_CENTRE_RATE := 4.0

const PAD_ACTIONS: Array[StringName] = [
	&"steer_axis_left", &"steer_axis_right", &"mainsheet_in", &"mainsheet_out", &"jib_sheet_in",
	&"jib_sheet_out", &"traveler_windward", &"traveler_leeward", &"backstay_pad_on",
	&"backstay_pad_off", &"jib_car_forward", &"jib_car_aft", &"orbit_left", &"orbit_right",
	&"orbit_up", &"orbit_down", &"zoom_in", &"zoom_out",
]

@export var boat_path: NodePath
## Cleared by the camera rig while the right stick orbits the camera, so it does not also trim the
## backstay. The backstay keys always work.
var pad_backstay_enabled := true

var _boat: Node


func _ready() -> void:
	_boat = get_node_or_null(boat_path)
	Input.joy_connection_changed.connect(_on_joy_connection_changed)


func _process(delta: float) -> void:
	if _boat == null:
		_boat = get_node_or_null(boat_path)
		if _boat == null:
			return

	_boat.tiller = _next_helm(_boat.tiller, delta)
	_boat.mainsheet = _held(_boat.mainsheet, &"mainsheet_in", &"mainsheet_out", SHEET_RATE * delta, 0.0, 1.0)

	var jib_in := Input.get_action_strength(&"jib_sheet_in")
	var jib_out := Input.get_action_strength(&"jib_sheet_out")
	if jib_in != jib_out:
		if _leeward_is_port():
			_boat.jib_sheet_port = clampf(_boat.jib_sheet_port + (jib_in - jib_out) * SHEET_RATE * delta, 0.0, 1.0)
		else:
			_boat.jib_sheet_starboard = clampf(_boat.jib_sheet_starboard + (jib_in - jib_out) * SHEET_RATE * delta, 0.0, 1.0)

	_boat.traveler = _held(_boat.traveler, &"traveler_leeward", &"traveler_windward", TRAVELER_RATE * delta, -1.0, 1.0)
	_boat.backstay = clampf(_boat.backstay + _backstay_push() * SHEET_RATE * delta, 0.0, 1.0)
	_boat.jib_car = _held(_boat.jib_car, &"jib_car_aft", &"jib_car_forward", SHEET_RATE * delta, 0.0, 1.0)

	if Input.is_action_just_pressed(&"reef_toggle"):
		_boat.reef = 0.0 if _boat.reef >= 0.5 else 1.0
	if Input.is_action_just_pressed(&"camera_switch"):
		camera_switch_requested.emit()
	if Input.is_action_just_pressed(&"drop_mark"):
		mark_drop_requested.emit()
	if Input.is_action_just_pressed(&"settings_toggle"):
		settings_toggled.emit()


func _held(value: float, increase: StringName, decrease: StringName, step: float, lo: float, hi: float) -> float:
	var push := Input.get_action_strength(increase) - Input.get_action_strength(decrease)
	return clampf(value + push * step, lo, hi)


func _backstay_push() -> float:
	var on := Input.get_action_strength(&"backstay_on")
	var off := Input.get_action_strength(&"backstay_off")
	if pad_backstay_enabled:
		on = maxf(on, Input.get_action_strength(&"backstay_pad_on"))
		off = maxf(off, Input.get_action_strength(&"backstay_pad_off"))
	return on - off


# The stick sets the tiller directly. Keys ramp it, and it self-centres once they are released, so
# a tap gives a small correction instead of leaving the helm over.
func _next_helm(current: float, delta: float) -> float:
	var stick := Input.get_axis(&"steer_axis_left", &"steer_axis_right")
	if stick != 0.0:
		return clampf(stick, -1.0, 1.0)
	var keys := Input.get_axis(&"steer_left", &"steer_right")
	if keys != 0.0:
		return clampf(current + keys * HELM_RAMP_RATE * delta, -1.0, 1.0)
	return move_toward(current, 0.0, HELM_CENTRE_RATE * delta)


# awa_deg is positive with the wind over the starboard side, so the jib is to port and the port
# sheet is the loaded one. Dead ahead counts as port so exactly one sheet is ever chosen.
func _leeward_is_port() -> bool:
	var state: Dictionary = _boat.get_state()
	return float(state.get("awa_deg", 0.0)) >= 0.0


# A pad that vanishes while a stick or button is down never sends the release, which would leave
# the action stuck on. An action still held by a key stays held.
func _on_joy_connection_changed(_device: int, connected: bool) -> void:
	if connected:
		return
	for action in PAD_ACTIONS:
		if Input.is_action_pressed(action) and not _held_by_key(action):
			Input.action_release(action)


func _held_by_key(action: StringName) -> bool:
	for event in InputMap.action_get_events(action):
		if event is InputEventKey and Input.is_physical_key_pressed(event.physical_keycode):
			return true
	return false
