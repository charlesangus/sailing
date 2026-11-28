class_name CameraRig
extends Node3D
## Chase, orbit and helm cameras for the boat. The camera-switch action of the BoatInput cycles
## them. The water follows whichever camera is current.

enum Mode { CHASE, ORBIT, HELM }

const ORBIT_YAW_RATE := 2.0
const ORBIT_PITCH_RATE := 1.0
const ZOOM_RATE := 1.2
const MOUSE_RAD_PER_PIXEL := 0.005
const WHEEL_ZOOM_STEP := 0.9
const LOOK_YAW_LIMIT := deg_to_rad(100.0)
const LOOK_PITCH_MIN := deg_to_rad(-40.0)
const LOOK_PITCH_MAX := deg_to_rad(30.0)
const HELM_SIDE_RATE := 2.0

@export var boat_path: NodePath
@export var input_path: NodePath
## The camera used by the chase and orbit modes.
@export var camera_path: NodePath

@export_group("Chase")
@export var chase_distance_m := 16.0
@export var chase_height_m := 5.0
@export var chase_target_height_m := 2.5
@export var chase_time_constant_s := 1.0

@export_group("Orbit")
@export var orbit_distance_m := 22.0
@export var orbit_min_distance_m := 6.0
@export var orbit_max_distance_m := 80.0
@export var orbit_pitch_deg := 20.0
@export var orbit_min_pitch_deg := 5.0
@export var orbit_max_pitch_deg := 85.0
@export var orbit_target_height_m := 1.5
@export var orbit_min_height_m := 0.5

@export_group("Helm")
## Hull-frame (x forward, y to port, z up) eye position: aft of the cockpit, at the tiller,
## 1 m above the deck. y is the windward offset and its sign follows the tack.
@export var helm_eye_x_m := -3.2
@export var helm_eye_side_m := 0.5
@export var helm_eye_z_m := 1.65
@export var helm_pitch_deg := -8.0
@export var helm_fov_deg := 80.0

var mode := Mode.CHASE
var orbit_distance := 0.0
var orbit_pitch := 0.0
var helm_camera: Camera3D

var _boat: Node3D
var _input: Node
var _camera: Camera3D
var _azimuth := 0.0
var _look_yaw := 0.0
var _look_pitch := 0.0
var _helm_side := 1.0
var _snap_pending := true


func _ready() -> void:
	# After the boat, so the cameras follow its pose of this frame.
	process_priority = 100
	_boat = get_node_or_null(boat_path)
	_camera = get_node_or_null(camera_path)
	if _boat == null or _camera == null:
		push_error("CameraRig: boat_path and camera_path must name nodes")
		set_process(false)
		set_process_unhandled_input(false)
		return
	_input = get_node_or_null(input_path)
	if _input != null:
		_input.camera_switch_requested.connect(cycle_mode)
	orbit_distance = orbit_distance_m
	orbit_pitch = deg_to_rad(orbit_pitch_deg)
	_build_helm_camera()
	_azimuth = _behind_azimuth()
	_apply_mode()
	_update_cameras(0.0)


func cycle_mode() -> void:
	set_mode((int(mode) + 1) % Mode.size() as Mode)


func set_mode(next: Mode) -> void:
	if next == Mode.HELM and helm_camera == null:
		next = Mode.CHASE
	if next == Mode.ORBIT:
		orbit_distance = orbit_distance_m
		orbit_pitch = deg_to_rad(orbit_pitch_deg)
	mode = next
	_look_yaw = 0.0
	_look_pitch = 0.0
	_apply_mode()
	_update_cameras(0.0)


func current_camera() -> Camera3D:
	return helm_camera if mode == Mode.HELM else _camera


func _process(delta: float) -> void:
	# The scene places the boat after this node is ready, so the first frame starts from its heading.
	if _snap_pending:
		_snap_pending = false
		_azimuth = _behind_azimuth()
	_update_cameras(delta)


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseMotion and (event.button_mask & MOUSE_BUTTON_MASK_RIGHT) != 0:
		if mode == Mode.ORBIT:
			_orbit_by(-event.relative.x * MOUSE_RAD_PER_PIXEL, event.relative.y * MOUSE_RAD_PER_PIXEL)
		elif mode == Mode.HELM:
			_look_by(-event.relative.x * MOUSE_RAD_PER_PIXEL, -event.relative.y * MOUSE_RAD_PER_PIXEL)
	elif event is InputEventMouseButton and event.pressed and mode == Mode.ORBIT:
		if event.button_index == MOUSE_BUTTON_WHEEL_UP:
			_zoom_by(WHEEL_ZOOM_STEP)
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			_zoom_by(1.0 / WHEEL_ZOOM_STEP)


func _apply_mode() -> void:
	if _input != null:
		# The right stick is the orbit control in orbit mode and the backstay otherwise.
		_input.pad_backstay_enabled = mode != Mode.ORBIT
	current_camera().make_current()


func _update_cameras(delta: float) -> void:
	match mode:
		Mode.CHASE:
			var alpha := 1.0 if delta <= 0.0 else 1.0 - exp(-delta / maxf(chase_time_constant_s, 0.01))
			_azimuth = lerp_angle(_azimuth, _behind_azimuth(), alpha)
			_place(chase_distance_m, chase_height_m, chase_target_height_m)
		Mode.ORBIT:
			var yaw := Input.get_action_strength(&"orbit_right") - Input.get_action_strength(&"orbit_left")
			var pitch := Input.get_action_strength(&"orbit_up") - Input.get_action_strength(&"orbit_down")
			_orbit_by(yaw * ORBIT_YAW_RATE * delta, pitch * ORBIT_PITCH_RATE * delta)
			var zoom := Input.get_action_strength(&"zoom_out") - Input.get_action_strength(&"zoom_in")
			_zoom_by(exp(zoom * ZOOM_RATE * delta))
			var height := maxf(orbit_target_height_m + sin(orbit_pitch) * orbit_distance, orbit_min_height_m)
			_place(cos(orbit_pitch) * orbit_distance, height, orbit_target_height_m)
		Mode.HELM:
			_update_helm(delta)


func _place(horizontal: float, height: float, target_height: float) -> void:
	var target := _boat.global_position + Vector3.UP * target_height
	var offset := Vector3(sin(_azimuth) * horizontal, height - target_height, cos(_azimuth) * horizontal)
	_camera.global_position = target + offset
	_camera.look_at(target, Vector3.UP)


func _update_helm(delta: float) -> void:
	var state: Dictionary = _boat.get_state()
	var windward := -1.0 if float(state.get("awa_deg", 0.0)) >= 0.0 else 1.0
	_helm_side = windward if delta <= 0.0 else move_toward(_helm_side, windward, HELM_SIDE_RATE * delta)
	helm_camera.position = Vector3(helm_eye_x_m, helm_eye_side_m * _helm_side, helm_eye_z_m)
	var pitch := deg_to_rad(helm_pitch_deg) + _look_pitch
	var direction := Vector3(cos(pitch) * cos(_look_yaw), cos(pitch) * sin(_look_yaw), sin(pitch))
	helm_camera.basis = Basis.looking_at(direction, Vector3(0.0, 0.0, 1.0))


func _orbit_by(yaw: float, pitch: float) -> void:
	_azimuth = wrapf(_azimuth + yaw, -PI, PI)
	orbit_pitch = clampf(orbit_pitch + pitch, deg_to_rad(orbit_min_pitch_deg), deg_to_rad(orbit_max_pitch_deg))


func _zoom_by(factor: float) -> void:
	orbit_distance = clampf(orbit_distance * factor, orbit_min_distance_m, orbit_max_distance_m)


func _look_by(yaw: float, pitch: float) -> void:
	_look_yaw = clampf(_look_yaw + yaw, -LOOK_YAW_LIMIT, LOOK_YAW_LIMIT)
	_look_pitch = clampf(_look_pitch + pitch, LOOK_PITCH_MIN, LOOK_PITCH_MAX)


# Azimuth of the horizontal direction from the boat to a point straight behind it. Heel and pitch
# never enter, so the chase view keeps the horizon level.
func _behind_azimuth() -> float:
	var forward := -_boat.global_transform.basis.z
	forward.y = 0.0
	if forward.length_squared() < 1e-6:
		forward = Vector3.FORWARD
	return atan2(-forward.x, -forward.z)


func _build_helm_camera() -> void:
	var visual := _boat.get_node_or_null("Visual")
	if visual == null or visual.get("hull_frame") == null:
		push_error("CameraRig: the boat has no built hull frame, so there is no helm camera")
		return
	helm_camera = Camera3D.new()
	helm_camera.name = "HelmCamera"
	helm_camera.fov = helm_fov_deg
	helm_camera.far = _camera.far
	visual.hull_frame.add_child(helm_camera)
