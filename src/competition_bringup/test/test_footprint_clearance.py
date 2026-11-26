import math
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from footprint_clearance import (ROBOT_FOOTPRINT, polygon_distance,
    rotated_polygon, square, validate_sweep)


def map_with_opening(lower=.875,upper=1.925):
    resolution=.025;width=240;height=160
    values=[0]*(width*height)
    for row in range(height):
        y=-1+(row+.5)*resolution
        for col in range(width):
            x=-1+(col+.5)*resolution
            if 1.95<=x<=2.05 and not lower<=y<=upper:
                values[row*width+col]=100
    return {'resolution':resolution,'width':width,'height':height,
            'origin':[-1.,-1.],'data':values}


class FootprintClearanceTests(unittest.TestCase):
    def test_nav2_polygon_encloses_actual_collision_extents(self):
        root=Path(__file__).resolve().parents[3]
        urdf=ET.parse(root/'src/competition_robot_description/urdf/pusher_robot.urdf.xacro').getroot()
        chassis=urdf.find(".//link[@name='base_link']/collision/geometry/box")
        pusher=urdf.find(".//link[@name='pusher_link']/collision/geometry/box")
        mount=urdf.find(".//joint[@name='pusher_joint']/origin")
        self.assertEqual(chassis.attrib['size'],'0.40 0.30 0.20')
        self.assertEqual(pusher.attrib['size'],'0.025 0.24 0.07')
        front=float(mount.attrib['xyz'].split()[0])+float(pusher.attrib['size'].split()[0])/2
        self.assertAlmostEqual(front,max(x for x,y in ROBOT_FOOTPRINT))
        self.assertAlmostEqual(.15+.035/2,max(abs(y) for x,y in ROBOT_FOOTPRINT))

    def test_urdf_footprint_extents_and_rotation(self):
        self.assertEqual(min(x for x,y in ROBOT_FOOTPRINT),-.20)
        self.assertEqual(max(x for x,y in ROBOT_FOOTPRINT),.2375)
        self.assertEqual(max(abs(y) for x,y in ROBOT_FOOTPRINT),.1675)
        self.assertAlmostEqual(max(math.hypot(*p) for p in ROBOT_FOOTPRINT),.2660944,places=5)
        p=rotated_polygon(ROBOT_FOOTPRINT,(0.,0.,math.radians(30)))
        self.assertAlmostEqual(max(y for x,y in p)-min(y for x,y in p),.475,places=3)

    def test_polygon_distance_detects_edge_and_corner_contact(self):
        self.assertEqual(polygon_distance(square(0,0,1),square(1,0,1)),0.)
        self.assertEqual(polygon_distance(square(0,0,1),square(1,1,1)),0.)
        self.assertAlmostEqual(polygon_distance(square(0,0,1),square(1.2,0,1)),.2)

    def test_aligned_passage_passes_while_disk_and_turn_fail(self):
        grid=map_with_opening()
        aligned=validate_sweep([(1.7,1.4,0.),(2.3,1.4,0.)],
                               {'map':grid},map_margin=.3126)
        self.assertTrue(aligned['safe'],aligned)
        self.assertGreater(aligned['minimum']['map'],.3126)
        rotated=validate_sweep([(1.7,1.4,0.),(2.,1.4,math.radians(30)),
                                (2.3,1.4,0.)],{'map':grid},map_margin=.3126)
        self.assertFalse(rotated['safe'])
        self.assertEqual(rotated['reason'],'map_clearance')
        # The older disk requires 0.37 + 0.3126 m on both sides of a
        # 1.05 m opening, which cannot fit despite the aligned polygon fit.
        self.assertGreater(2*(.37+.3126),1.05)
        uncertain=validate_sweep([(1.7,1.4,0.),(2.3,1.4,0.)],
                                  {'map':grid},map_margin=.3183,
                                  yaw_uncertainty=.28)
        self.assertFalse(uncertain['safe'])
        self.assertNotEqual(uncertain['yaw_offset_rad'],0.)

    def test_cube_between_approach_lanes_is_not_deleted(self):
        grid=map_with_opening()
        cubes=[('green',square(.71,1.31,.18)),
               ('blue',square(.71,.11,.18))]
        path=[(.4,.8,0.),(1.2,.8,0.)]
        result=validate_sweep(path,{'map':grid},cubes=cubes,
                              map_margin=.3126,cube_margin=.2626)
        self.assertTrue(result['safe'],result)
        self.assertLess(result['minimum']['cube'],.37+.1273+.2626)
        blocked=validate_sweep([(.4,1.35,0.),(1.2,1.35,0.)],
                               {'map':grid},cubes=cubes,cube_margin=.2626)
        self.assertFalse(blocked['safe'])
        self.assertEqual(blocked['reason'],'cube_clearance')

    def test_unknown_cell_and_single_pose_are_checked(self):
        grid=map_with_opening()
        col=int((2.3-grid['origin'][0])/grid['resolution'])
        row=int((1.4-grid['origin'][1])/grid['resolution'])
        grid['data'][row*grid['width']+col]=-1
        result=validate_sweep([(2.3,1.4,0.)],{'map':grid},map_margin=.1)
        self.assertFalse(result['safe'])
        self.assertEqual(result['reason'],'map_clearance')


if __name__=='__main__':
    unittest.main()
