"""Extract TCC1 main-product solids, STL and unit-mass properties from the supplied STEP archive."""

import argparse
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from zipfile import ZipFile

from OCP.Bnd import Bnd_Box
from OCP.BRep import BRep_Builder
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.GProp import GProp_GProps
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Compound
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.collections import Sequence_TDF_Label


PACKAGE = Path(__file__).resolve().parents[1] / "src/tcc1_tool_changer_description"
MASTER_FLANGE_Z_MM = 59.5154724193
TOOL_OUTPUT_Z_MM = -37.4845275807
TOOL_SHIFT_Z_MM = 55.0
GROUPS = {
    "master": ("TCC1_MASTER_AIR_POGO_COUPLER_ASSY.stp", "5K_TCC1_MASTER_20240724", 32),
    "tool": ("TCC1_TOOL_AIR_POGO_COUPLER_ASSY.stp", "5K_TCC1_TOOL_PLATE_20240724", 5),
}


def bounds(shape):
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box, False, False)
    return box.CornerMin().Coord(), box.CornerMax().Coord()


def extraction(step: Path, part: str, group: str, expected_count: int) -> dict:
    doc = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.ReadFile(str(step))
    if not reader.Transfer(doc):
        raise RuntimeError(f"Unable to transfer {step}")
    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots, children = Sequence_TDF_Label(), Sequence_TDF_Label()
    shape_tool.GetFreeShapes(roots)
    shape_tool.GetComponents_s(roots.Value(1), children, False)
    compound, builder = TopoDS_Compound(), BRep_Builder()
    builder.MakeCompound(compound)
    properties, kept, extensions = GProp_GProps(), 0, []
    for index in range(1, children.Length() + 1):
        child, ref, name = children.Value(index), TDF_Label(), TDataStd_Name()
        shape_tool.GetReferredShape_s(child, ref)
        ref.FindAttribute(TDataStd_Name.GetID_s(), name)
        if name.Get().ToExtString() != group:
            continue
        solids = TopExp_Explorer(shape_tool.GetShape_s(child), TopAbs_SOLID)
        while solids.More():
            solid = TopoDS.Solid(solids.Current())
            solids.Next()
            lo, hi = bounds(solid)
            if part == "tool" and hi[2] <= TOOL_OUTPUT_Z_MM + 1e-6:
                extensions.append({"bbox_min_mm": lo, "bbox_max_mm": hi})
                continue
            volume = GProp_GProps()
            BRepGProp.VolumeProperties_s(solid, volume, True)
            properties.Add(volume)
            builder.Add(compound, solid)
            kept += 1
    if kept != expected_count or (part == "tool" and len(extensions) != 1):
        raise ValueError(f"Unexpected {part} STEP scope: {kept} solids, {len(extensions)} extensions")
    BRepMesh_IncrementalMesh(compound, 0.08, False, 0.30)
    mesh = PACKAGE / "meshes" / f"tcc1_{part}_nominal_mm.stl"
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    if not writer.Write(compound, str(mesh)):
        raise RuntimeError(f"Unable to write {mesh}")
    lo, hi = bounds(compound)
    matrix = properties.MatrixOfInertia()
    return {
        "source_file": step.name, "source_sha256": hashlib.sha256(step.read_bytes()).hexdigest(),
        "group": group, "solid_count": kept, "excluded_output_extensions": extensions,
        "mesh": mesh.relative_to(PACKAGE).as_posix(), "mesh_sha256": hashlib.sha256(mesh.read_bytes()).hexdigest(),
        "bbox_min_mm": lo, "bbox_max_mm": hi, "sum_volume_mm3": properties.Mass(),
        "uniform_volume_centroid_mm": properties.CentreOfMass().Coord(),
        "uniform_volume_inertia_per_unit_mass_m2": [
            [matrix.Value(i, j) / properties.Mass() * 1e-6 for j in (1, 2, 3)] for i in (1, 2, 3)
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("step_zip", type=Path)
    args = parser.parse_args()
    manifest = {
        "method": "Sum of selected STEP solid volumes, uniform density, CAD axes, tensor about CAD CoM.",
        "mount_registration_mm": {
            "master_flange_z": MASTER_FLANGE_Z_MM, "tool_raw_z_shift": TOOL_SHIFT_Z_MM,
            "tool_output_z": TOOL_OUTPUT_Z_MM,
            "coupled_height": round(MASTER_FLANGE_Z_MM - TOOL_SHIFT_Z_MM - TOOL_OUTPUT_Z_MM, 8),
        },
        "parts": {},
    }
    with ZipFile(args.step_zip) as archive, TemporaryDirectory() as folder:
        for part, (filename, group, count) in GROUPS.items():
            step = Path(folder) / filename
            step.write_bytes(archive.read(filename))
            manifest["parts"][part] = extraction(step, part, group, count)
    (PACKAGE / "config/nominal_cad_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
