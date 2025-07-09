# FreeCAD URDF Exporter

This project exports a FreeCAD Assembly4 robot to URDF, including mesh and inertial data, using Python scripts and FreeCAD's Python API.

## How to Run as Macro
### Prerequisites
- FreeCAD (with Python scripting enabled)
- Assembly4 workbench (for robot assemblies)
- Python modules: `importlib`, `os`, `Mesh`, `FreeCAD`
- Place this repo in your FreeCAD macros directory or a known path

### 1. Open FreeCAD in GUI 

### 2. Use the Macro Control Panel (MCP) or FreeCAD Python Console

## How to Run Via MCP (For Pair Development with AI assistant in Cursor etc)
### Prerequisites
- Same as above
- PLUS `freecadmcp`
- Add the MCP config to your tool of choice (e.g. Cursor) via the MCP settings
- Open the FreeCAD MCP workbench in FreeCAD
- Ask your AI to pass the contents of `run_in_freecad.py` (this file should not be run directly, but its code contents is the `code` param for the MCP call)

Your model should request to make this MCP call
```
{
  "code": "import importlib\nimport freecad_helpers\nimportlib.reload(freecad_helpers)\nimport ExportAssembly4ToURDF_tree\nimportlib.reload(ExportAssembly4ToURDF_tree)\nExportAssembly4ToURDF_tree.main()\n"
}
```

- All imported files should be included in `importlib.reload(filename)` calls in the `code` param so that code changes are picked up (otherwise modules may be cached)
