extends SceneTree

const FRAMES := 5


func _fail(message: String) -> void:
	printerr("FAIL: " + message)
	quit(1)


func _initialize() -> void:
	var shader: Shader = load("res://shaders/flat_water.gdshader")
	if shader == null or shader.code.is_empty():
		_fail("flat_water.gdshader did not load")
		return
	var env_packed: PackedScene = load("res://scenes/environment.tscn")
	var main_packed: PackedScene = load("res://scenes/main.tscn")
	if env_packed == null or main_packed == null:
		_fail("cannot load environment.tscn or main.tscn")
		return

	var lone = env_packed.instantiate()
	root.add_child(lone)
	for i in FRAMES:
		await process_frame
	var lone_water: Water = lone.get_node("Water")
	if lone_water.depth_image != null:
		_fail("environment.tscn alone should not find a terrain")
		return
	lone.queue_free()
	await process_frame

	var main = main_packed.instantiate()
	root.add_child(main)
	for i in FRAMES:
		await process_frame

	var water: Water = main.get_node("Environment/Water")
	var camera: Camera3D = main.get_node("Camera")
	if water.get_viewport().get_camera_3d() == null:
		print("no current camera in headless viewport; making the follow camera current")
		camera.make_current()
		await process_frame
	if not is_zero_approx(water.global_position.y):
		_fail("water is at y = %f, want 0" % water.global_position.y)
		return
	if water.depth_image == null or water.depth_image.get_width() != water.depth_resolution \
			or water.depth_image.get_height() != water.depth_resolution:
		_fail("depth image missing or not %d square" % water.depth_resolution)
		return
	var mat := water.material_override as ShaderMaterial
	if mat.get_shader_parameter("depth_tex") != water.depth_texture:
		_fail("shader is not given the depth texture")
		return

	var terrain = main.get_node("BayTerrain")
	var start: Vector3 = terrain.start_position_godot()
	var shallow := Vector3.ZERO
	var deep := Vector3.ZERO
	var best_shallow := INF
	var best_deep := 0.0
	for z in range(-1900, 1900, 40):
		for x in range(-1900, 1900, 40):
			var d := water.depth_at(Vector3(x, 0, z))
			if d > 0.0 and d < best_shallow:
				best_shallow = d
				shallow = Vector3(x, 0, z)
			if d > best_deep:
				best_deep = d
				deep = Vector3(x, 0, z)
	print("depth: shallowest water %.2f m, deepest %.2f m, start %.2f m" % [
		best_shallow, best_deep, water.depth_at(start)])
	if not (best_shallow < 3.0 and best_deep > 15.0):
		_fail("depth texture lacks a shallow/deep contrast")
		return
	if water.depth_at(shallow) >= water.depth_at(deep):
		_fail("shallow point is not shallower than deep point")
		return

	main.set_process(false)
	camera.global_position = Vector3(1000.0, 5.0, -700.0)
	await process_frame
	await process_frame
	var expected := Vector3(snappedf(1000.0, water.snap_m), 0.0, snappedf(-700.0, water.snap_m))
	if water.global_position.distance_to(expected) > 0.01:
		_fail("water at %s did not follow the camera to %s" % [water.global_position, expected])
		return

	print("PASS")
	quit(0)
