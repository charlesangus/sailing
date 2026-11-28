extends SceneTree

const SCENE_PATH := "res://scenes/bay_terrain.tscn"
const MAX_BUILD_MSEC := 5000


func _fail(message: String) -> void:
	printerr("FAIL: " + message)
	quit(1)


func _initialize() -> void:
	var packed: PackedScene = load(SCENE_PATH)
	if packed == null:
		_fail("cannot load " + SCENE_PATH)
		return
	var terrain = packed.instantiate()
	root.add_child(terrain)
	await process_frame
	if not terrain.is_built():
		_fail("terrain did not build: " + terrain.last_error)
		return

	var step_px := roundi(terrain.grid_step_m / terrain.pixel_size_m)
	var expected_columns: int = (terrain.width_px - 1) / step_px + 1
	var expected_rows: int = (terrain.height_px - 1) / step_px + 1
	var mesh: ArrayMesh = terrain.mesh_instance.mesh
	var arrays := mesh.surface_get_arrays(0)
	var vertex_count: int = arrays[Mesh.ARRAY_VERTEX].size()
	var index_count: int = arrays[Mesh.ARRAY_INDEX].size()
	print("terrain %d x %d px -> %d x %d grid, %d vertices, %d triangles, built in %d ms" % [
		terrain.width_px, terrain.height_px, terrain.grid_columns, terrain.grid_rows,
		vertex_count, index_count / 3, terrain.build_msec])
	if vertex_count != expected_columns * expected_rows:
		_fail("vertex count %d, want %d" % [vertex_count, expected_columns * expected_rows])
		return
	if index_count != (expected_columns - 1) * (expected_rows - 1) * 6:
		_fail("index count %d does not match the grid" % index_count)
		return
	if terrain.build_msec > MAX_BUILD_MSEC:
		_fail("build took %d ms, want <= %d" % [terrain.build_msec, MAX_BUILD_MSEC])
		return

	var start: Dictionary = terrain.meta["start"]
	var start_z: float = terrain.height_at_world(float(start["x_m"]), float(start["y_m"]))
	print("seabed at the start (%.0f, %.0f): %.2f m" % [float(start["x_m"]), float(start["y_m"]), start_z])
	if start_z > -5.0:
		_fail("seabed at the start is %.2f m, want at least 5 m deep" % start_z)
		return
	var start_godot: Vector3 = terrain.start_position_godot()
	if not is_equal_approx(terrain.height_at_godot(start_godot), start_z):
		_fail("height_at_godot disagrees with height_at_world at the start")
		return

	# Vertex 0 is pixel (0, 0): the image's north-west corner, at Godot -Z.
	var first: Vector3 = arrays[Mesh.ARRAY_VERTEX][0]
	var origin := Vector3(terrain.origin_x_m, terrain.height_at_world(terrain.origin_x_m, terrain.origin_y_m), -terrain.origin_y_m)
	if not first.is_equal_approx(origin):
		_fail("vertex 0 is %s, want %s" % [first, origin])
		return

	var highest: float = arrays[Mesh.ARRAY_VERTEX][0].y
	for v: Vector3 in arrays[Mesh.ARRAY_VERTEX]:
		highest = maxf(highest, v.y)
	if highest <= 10.0:
		_fail("highest terrain vertex is %.2f m, want land above 10 m" % highest)
		return

	terrain.queue_free()
	print("PASS")
	quit(0)
