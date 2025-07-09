import importlib
import freecad_helpers
importlib.reload(freecad_helpers)
import logging_utils
importlib.reload(logging_utils)
import ExportAssembly4ToURDF_tree
importlib.reload(ExportAssembly4ToURDF_tree)    
ExportAssembly4ToURDF_tree.main()
