extends SceneTree


func _fail(message: String) -> void:
	printerr("FAIL: " + message)
	quit(1)


func _initialize() -> void:
	var boat = ClassDB.instantiate("SailBoat")
	boat.boat_dir = "res://data/boats/no_such_boat"
	var error: String = boat.get_last_error()
	var active: bool = boat.is_active()
	var stepped: int = boat.step_seconds(1.0)
	var reset: bool = boat.reset_boat(Vector3.ZERO, 0.0)
	var state: Dictionary = boat.get_state()
	boat.free()

	if active or error.find("no_such_boat") < 0:
		_fail("a missing boat dir left active=%s, error '%s'" % [active, error])
		return
	if stepped != 0 or reset or state.get("active", true):
		_fail("an inert boat stepped %d, reset %s, state %s" % [stepped, reset, state])
		return

	print("PASS")
	quit(0)
