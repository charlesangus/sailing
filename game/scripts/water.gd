class_name Water
extends MeshInstance3D

## The flat sea at y = 0. It follows the camera horizontally in whole grid steps and feeds the
## shader a depth texture baked from the terrain's bathymetry.

const DEPTH_SHADER_RECT := "depth_rect"

@export var terrain_path: NodePath
@export var snap_m := 64.0
@export var depth_resolution := 256

var depth_image: Image
var depth_texture: ImageTexture
## World x, z of the depth texture's min corner and its extent in metres.
var depth_rect := Rect2()

var _terrain: Node
var _material: ShaderMaterial


func _ready() -> void:
	# After the camera has moved this frame.
	process_priority = 200
	top_level = true
	_material = material_override as ShaderMaterial
	_terrain = _find_terrain()
	if _terrain != null and _terrain.is_built():
		bake_depth(_terrain)
	_follow_camera()


func _process(_delta: float) -> void:
	_follow_camera()


## Bakes water depth in metres (0 on land) into a low-resolution float texture. Pixel i covers
## the world span [i, i + 1] / resolution of the terrain's extent, sampled at its centre.
func bake_depth(terrain: Node) -> void:
	var min_x: float = terrain.origin_x_m
	var min_z: float = -terrain.origin_y_m
	var size_x: float = (terrain.width_px - 1) * terrain.pixel_size_m
	var size_z: float = (terrain.height_px - 1) * terrain.pixel_size_m
	depth_rect = Rect2(min_x, min_z, size_x, size_z)
	depth_image = Image.create(depth_resolution, depth_resolution, false, Image.FORMAT_RF)
	for row in depth_resolution:
		var z := min_z + (row + 0.5) / depth_resolution * size_z
		for col in depth_resolution:
			var x := min_x + (col + 0.5) / depth_resolution * size_x
			var height: float = terrain.height_at_godot(Vector3(x, 0.0, z))
			depth_image.set_pixel(col, row, Color(maxf(-height, 0.0), 0.0, 0.0))
	depth_texture = ImageTexture.create_from_image(depth_image)
	if _material != null:
		_material.set_shader_parameter("depth_tex", depth_texture)
		_material.set_shader_parameter(DEPTH_SHADER_RECT,
			Vector4(depth_rect.position.x, depth_rect.position.y, depth_rect.size.x, depth_rect.size.y))


## Baked depth in metres at a Godot-frame point; open-sea depth is not applied outside the map.
func depth_at(point: Vector3) -> float:
	if depth_image == null or not depth_rect.has_point(Vector2(point.x, point.z)):
		return -1.0
	var col := clampi(int((point.x - depth_rect.position.x) / depth_rect.size.x * depth_resolution), 0, depth_resolution - 1)
	var row := clampi(int((point.z - depth_rect.position.y) / depth_rect.size.y * depth_resolution), 0, depth_resolution - 1)
	return depth_image.get_pixel(col, row).r


func _follow_camera() -> void:
	var camera := get_viewport().get_camera_3d()
	if camera == null:
		return
	var at := camera.global_position
	global_position = Vector3(snappedf(at.x, snap_m), 0.0, snappedf(at.z, snap_m))


func _find_terrain() -> Node:
	if not terrain_path.is_empty():
		return get_node_or_null(terrain_path)
	var ancestor := get_parent()
	while ancestor != null:
		for child in ancestor.get_children():
			if child is TerrainBuilder:
				return child
		ancestor = ancestor.get_parent()
	return null
