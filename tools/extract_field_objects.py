import argparse
import json
from pathlib import Path

import trimesh


def _iter_node_geometry_pairs(scene: trimesh.Scene):
	"""
	Yield (node_name, geom_name) pairs across trimesh versions.

	Some trimesh versions expose scene.graph.nodes_geometry as:
	- dict[node_name] = geom_name
	Others expose:
	- list[node_name] where node_name == geom_name for geometry nodes
	"""
	nodes_geometry = scene.graph.nodes_geometry

	if isinstance(nodes_geometry, dict):
		for node_name, geom_name in nodes_geometry.items():
			yield node_name, geom_name
		return

	if isinstance(nodes_geometry, list):
		for node_name in nodes_geometry:
			# In this API shape, geometry nodes are typically named the same as geometry keys.
			geom_name = node_name if node_name in scene.geometry else None
			if geom_name is None:
				continue
			yield node_name, geom_name
		return

	raise TypeError(f"Unsupported nodes_geometry type: {type(nodes_geometry)}")


def extract_objects(glb_path: Path, name_filter: str | None = None):
	"""
	Load a GLB scene and return world-space centers and AABBs for each geometry.
	"""
	scene = trimesh.load(glb_path, force="scene")
	if not isinstance(scene, trimesh.Scene):
		raise RuntimeError(f"{glb_path} did not load as a Scene; got {type(scene)} instead.")

	results = []

	for node_name, geom_name in _iter_node_geometry_pairs(scene):
		if name_filter and name_filter.lower() not in geom_name.lower():
			continue

		geom = scene.geometry[geom_name]
		transform, _ = scene.graph.get(frame_to=node_name)

		mesh_world = geom.copy()
		mesh_world.apply_transform(transform)

		min_bounds, max_bounds = mesh_world.bounds
		center = (min_bounds + max_bounds) / 2.0

		results.append(
			{
				"node": node_name,
				"name": geom_name,
				"center": center.tolist(),  # [x, y, z]
				"min_bounds": min_bounds.tolist(),
				"max_bounds": max_bounds.tolist(),
			}
		)

	return results


def main():
	parser = argparse.ArgumentParser(
		description="Extract world-space centers and bounds from a GLB field file."
	)
	parser.add_argument("glb", type=Path, help="Path to .glb field file")
	parser.add_argument(
		"--filter",
		type=str,
		default=None,
		help="Optional substring to filter geometry names (e.g. 'hub', 'tower', 'depot')",
	)
	parser.add_argument(
		"--pretty",
		action="store_true",
		help="Pretty-print JSON instead of compact output",
	)
	parser.add_argument(
		"--out",
		type=Path,
		default=None,
		help="Optional output JSON path (writes file in addition to stdout).",
	)
	parser.add_argument(
		"--quiet",
		action="store_true",
		help="Suppress JSON stdout (useful with --out).",
	)

	args = parser.parse_args()

	results = extract_objects(args.glb, args.filter)

	if args.pretty:
		output = json.dumps(results, indent=2)
	else:
		output = json.dumps(results)

	if not args.quiet:
		print(output)

	if args.out is not None:
		args.out.parent.mkdir(parents=True, exist_ok=True)
		args.out.write_text(output, encoding="utf-8")
		print(f"Wrote {len(results)} objects to {args.out}")

	if not results:
		print("No geometries matched your filter.", flush=True)


if __name__ == "__main__":
	main()

