"""
FreeCAD assembly for URDF export.
FreeCADAssembly flattens assemblies/parts into a list of FreeCADLink instances.
"""
from freecad_helpers import FreeCADLink


def _is_body_or_link_to_body(obj):
    """True if obj is a PartDesign::Body or App::Link chain ending in PartDesign::Body."""
    t = getattr(obj, "TypeId", "")
    lo = getattr(obj, "LinkedObject", None)
    if t == "PartDesign::Body":
        return True, obj
    if t == "App::Link" and lo:
        target = lo
        while target and getattr(target, "TypeId", "") == "App::Link":
            target = getattr(target, "LinkedObject", None)
        if target and getattr(target, "TypeId", "") == "PartDesign::Body":
            return True, obj
    return False, None


class FreeCADAssembly:
    """Wraps a FreeCAD assembly or part. Flattens to FreeCADLink list."""

    def __init__(self, assembly_obj):
        self.obj = assembly_obj

    def get_links(self):
        """Return a flattened list of FreeCADLink instances for all parts in this assembly and subassemblies."""
        result = []
        seen = set()

        def visit(obj):
            if obj is None:
                return
            if id(obj) in seen:
                return
            seen.add(id(obj))
            ok, part = _is_body_or_link_to_body(obj)
            if ok:
                result.append(FreeCADLink(part))
                return
            for child in getattr(obj, "Group", []) or []:
                visit(child)
            lo = getattr(obj, "LinkedObject", None)
            if lo:
                visit(lo)
                for c in getattr(lo, "Group", []) or []:
                    visit(c)

        for child in getattr(self.obj, "Group", []) or []:
            visit(child)
        return result
