class_name Marks
extends Node3D
## The windward and leeward race marks. Dropping places the windward mark upwind of the boat and the
## leeward mark at the boat, replacing the previous pair.

const WINDWARD_DISTANCE_M := 926.0
const BOB_AMPLITUDE_M := 0.12
const TILT_AMPLITUDE_RAD := 0.07

@export var boat_path: NodePath
@export var terrain_path: NodePath
@export var input_path: NodePath
@export var buoy_scene: PackedScene

var windward: Node3D
var leeward: Node3D

var _boat: Node
var _terrain: Node
var _time := 0.0


func _ready() -> void:
	_boat = get_node_or_null(boat_path)
	_terrain = get_node_or_null(terrain_path)
	var input := get_node_or_null(input_path)
	if input != null:
		input.mark_drop_requested.connect(drop_marks)


func _process(delta: float) -> void:
	_time += delta
	for mark in [windward, leeward]:
		if mark != null:
			_sway(mark)


## Returns false when there is no boat to place the marks from.
func drop_marks() -> bool:
	if _boat == null or buoy_scene == null:
		push_error("Marks: no boat or buoy scene")
		return false
	var origin: Vector3 = _boat.global_position
	origin.y = 0.0
	var bearing := deg_to_rad(float(_boat.wind_direction_deg))
	var upwind := Vector3(sin(bearing), 0.0, -cos(bearing))
	_clear()
	leeward = _place(origin, "Leeward")
	windward = _place(origin + upwind * WINDWARD_DISTANCE_M, "Windward")
	return true


func _place(at: Vector3, label: String) -> Node3D:
	if _terrain != null and _terrain.is_built() and _terrain.height_at_godot(at) >= 0.0:
		print("Marks: the %s mark at (%.0f, %.0f) is on land; placed there anyway" % [label.to_lower(), at.x, at.z])
	var mark: Node3D = buoy_scene.instantiate()
	mark.name = label
	add_child(mark)
	mark.global_position = at
	return mark


func _clear() -> void:
	for mark in [windward, leeward]:
		if mark != null:
			remove_child(mark)
			mark.queue_free()
	windward = null
	leeward = null


# Only the Body child moves, so the buoy node itself stays on the water at y = 0.
func _sway(mark: Node3D) -> void:
	var body: Node3D = mark.get_node("Body")
	var phase := mark.global_position.x * 0.013 + mark.global_position.z * 0.017
	body.position.y = sin(_time * 1.7 + phase) * BOB_AMPLITUDE_M
	body.rotation = Vector3(sin(_time * 1.1 + phase) * TILT_AMPLITUDE_RAD, 0.0, sin(_time * 1.3 + phase * 2.0) * TILT_AMPLITUDE_RAD)
