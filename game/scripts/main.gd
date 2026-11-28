extends Node3D
## The playable scene: places the boat at the bay's start and keeps the placeholder camera behind it.

@export var boat_path: NodePath
@export var terrain_path: NodePath
@export var camera_path: NodePath
@export var camera_distance_m := 16.0
@export var camera_height_m := 5.0
@export var camera_target_height_m := 2.5

var _boat: Node3D
var _camera: Camera3D


func _ready() -> void:
	# After the boat, so the camera follows its pose of this frame.
	process_priority = 100
	_boat = get_node(boat_path)
	_camera = get_node(camera_path)
	var terrain: Node = get_node(terrain_path)
	if not terrain.is_built():
		push_error("Main: the bay terrain did not build: " + terrain.last_error)
		return
	var start: Dictionary = terrain.meta["start"]
	var at := Vector3(float(start["x_m"]), 0.0, -float(start["y_m"]))
	if not _boat.reset_boat(at, float(start["heading_deg"])):
		push_error("Main: cannot place the boat: " + _boat.get_last_error())
	_follow()


func _process(_delta: float) -> void:
	_follow()


func _follow() -> void:
	var forward := -_boat.global_transform.basis.z
	forward.y = 0.0
	if forward.length_squared() < 1e-6:
		forward = Vector3.FORWARD
	forward = forward.normalized()
	var target := _boat.global_position + Vector3.UP * camera_target_height_m
	_camera.global_position = _boat.global_position - forward * camera_distance_m + Vector3.UP * camera_height_m
	_camera.look_at(target)
