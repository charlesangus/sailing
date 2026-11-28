extends SceneTree

const ANGLE_TOLERANCE_DEG := 0.5
const MAINSHEET_EASED := 0.8


func _fail(message: String) -> void:
	printerr("FAIL: " + message)
	quit(1)


func _vertex_count(node: MeshInstance3D) -> int:
	var mesh := node.mesh
	if mesh is ArrayMesh:
		return (mesh as ArrayMesh).surface_get_arrays(0)[Mesh.ARRAY_VERTEX].size()
	return mesh.get_faces().size()


# Rotation of a hull-frame node about the hull's vertical, deg.
func _yaw_deg(node: Node3D) -> float:
	return rad_to_deg(node.basis.get_euler(EULER_ORDER_ZYX).z)


func _initialize() -> void:
	var main_packed: PackedScene = load("res://scenes/main.tscn")
	var boat_packed: PackedScene = load("res://scenes/boat.tscn")
	if main_packed == null or boat_packed == null:
		_fail("cannot load boat.tscn or main.tscn")
		return

	var boat = boat_packed.instantiate()
	root.add_child(boat)
	await process_frame
	var visual: BoatVisual = boat.get_node("Visual")
	if not visual.is_built():
		_fail("visual not built: " + visual.last_error)
		return
	for node in [visual.hull_mesh, visual.keel_mesh, visual.rudder_mesh, visual.main_sail,
			visual.jib_sail, visual.mast_mesh, visual.tiller_mesh]:
		if _vertex_count(node) <= 0:
			_fail("no vertices in " + node.name)
			return

	boat.mainsheet = MAINSHEET_EASED
	boat.jib_sheet_port = 0.3
	boat.jib_sheet_starboard = 0.3
	boat.tiller = 0.6
	boat.step_seconds(10.0)
	visual.update_from_state()
	var state: Dictionary = boat.get_state()

	var boom_deg := _yaw_deg(visual.boom)
	if absf(absf(boom_deg) - absf(state["boom_angle_deg"])) > ANGLE_TOLERANCE_DEG or absf(state["boom_angle_deg"]) < 1.0:
		_fail("boom %f deg does not follow boom_angle_deg %f" % [boom_deg, state["boom_angle_deg"]])
		return
	# Positive chord angle has the clew to port, which is a negative rotation about the hull's up.
	if signf(boom_deg) == signf(state["boom_angle_deg"]):
		_fail("boom turns the wrong way: %f for %f" % [boom_deg, state["boom_angle_deg"]])
		return
	var main_deg := _yaw_deg(visual.main_sail)
	if absf(main_deg - boom_deg) > ANGLE_TOLERANCE_DEG:
		_fail("main sail %f deg does not follow the boom %f" % [main_deg, boom_deg])
		return

	var tack: Vector3 = visual._jib_geometry["tack"]
	var clew: Vector3 = visual._jib_geometry["clew"] - tack
	var turned: Vector3 = visual.jib_sail.basis * clew
	var jib_chord_deg := rad_to_deg(atan2(turned.y, -turned.x))
	if absf(jib_chord_deg - state["jib_angle_deg"]) > ANGLE_TOLERANCE_DEG:
		_fail("jib chord %f deg, jib_angle_deg %f" % [jib_chord_deg, state["jib_angle_deg"]])
		return

	var rudder_deg := rad_to_deg(visual.rudder_pivot.basis.get_euler(EULER_ORDER_ZYX).z)
	if absf(rudder_deg - state["rudder_angle_deg"]) > ANGLE_TOLERANCE_DEG or absf(rudder_deg) < 0.5:
		_fail("rudder %f deg, rudder_angle_deg %f" % [rudder_deg, state["rudder_angle_deg"]])
		return

	var hull_origin: Vector3 = visual.hull_frame.position
	if hull_origin.distance_to(state["hull_origin_local"]) > 1e-4 or hull_origin.length() > 3.0:
		_fail("hull frame offset %s is not the CG offset" % hull_origin)
		return
	print("boom %.2f deg for %.2f, jib chord %.2f for %.2f, rudder %.2f for %.2f" % [
		boom_deg, state["boom_angle_deg"], jib_chord_deg, state["jib_angle_deg"], rudder_deg,
		state["rudder_angle_deg"]])
	boat.queue_free()
	await process_frame

	var main = main_packed.instantiate()
	root.add_child(main)
	await process_frame
	await process_frame
	var main_boat: Node3D = main.get_node("Boat")
	var terrain = main.get_node("BayTerrain")
	if not main_boat.is_active():
		_fail("main boat inactive: " + main_boat.get_last_error())
		return
	var start: Dictionary = terrain.meta["start"]
	var position := main_boat.global_position
	if absf(position.x - start["x_m"]) > 1.0 or absf(position.z + start["y_m"]) > 1.0:
		_fail("boat at %s, start %s" % [position, start])
		return
	print("PASS")
	quit(0)
