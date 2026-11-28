extends Node3D
## The playable scene: places the boat at the bay's start.

@export var boat_path: NodePath
@export var terrain_path: NodePath

var _boat: Node3D


func _ready() -> void:
	_boat = get_node(boat_path)
	var terrain: Node = get_node(terrain_path)
	if not terrain.is_built():
		push_error("Main: the bay terrain did not build: " + terrain.last_error)
		return
	var start: Dictionary = terrain.meta["start"]
	var at := Vector3(float(start["x_m"]), 0.0, -float(start["y_m"]))
	if not _boat.reset_boat(at, float(start["heading_deg"])):
		push_error("Main: cannot place the boat: " + _boat.get_last_error())
