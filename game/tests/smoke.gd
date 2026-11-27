extends SceneTree


func _fail(message: String) -> void:
	printerr("FAIL: " + message)
	quit(1)


func _initialize() -> void:
	if not ClassDB.class_exists("SailSim"):
		_fail("class SailSim is not registered; the GDExtension did not load")
		return

	var expected := OS.get_environment("SAIL_EXPECTED_VERSION")
	if expected.is_empty():
		_fail("SAIL_EXPECTED_VERSION is not set")
		return

	var sim = ClassDB.instantiate("SailSim")
	if sim == null:
		_fail("ClassDB.instantiate(\"SailSim\") returned null")
		return

	var actual: String = sim.get_core_version()
	sim.free()
	if actual != expected:
		_fail("core version '%s' does not match expected '%s'" % [actual, expected])
		return

	print("PASS")
	quit(0)
