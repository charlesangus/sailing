extends Node3D
## The playable scene: places the boat at the bay's start.

@export var boat_path: NodePath
@export var terrain_path: NodePath

## Sheet settings for the start (TWA 90, 10 kn, starboard tack): the settled TWS 10, TWA 90 row of
## data/boats/j80/sim_polar.csv. The wind comes from the east in boat.tscn, so the start heading
## north is a beam reach through the harbour mouth.
const START_TRIM := {
	"mainsheet": 0.7659,
	"traveler": 0.0557,
	"backstay": 0.0,
	"jib_sheet_port": 0.618,
	"jib_sheet_starboard": 0.618,
	"jib_car": 0.4638,
}

## BSP of the same polar row, 6.30 kn, so the boat starts at its settled speed.
const START_SPEED_MPS := 3.24

var _boat: Node3D


func _ready() -> void:
	_boat = get_node(boat_path)
	var terrain: Node = get_node(terrain_path)
	if not terrain.is_built():
		push_error("Main: the bay terrain did not build: " + terrain.last_error)
		return
	var heading_deg := float(terrain.meta["start"]["heading_deg"])
	if not _boat.reset_boat(terrain.start_position_godot(), heading_deg, START_SPEED_MPS):
		push_error("Main: cannot place the boat: " + _boat.get_last_error())
		return
	for key in START_TRIM:
		_boat.set(key, START_TRIM[key])
