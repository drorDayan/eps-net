import CGALPY.Arr2 as Arr2
import CGALPY.Ker as KER
import CGALPY.Pol2 as POL2
import CGALPY.MN2 as MN2

FT = KER.FT

class Collision_detector:
    cspace = None
    pl = None

    def __init__(self, obstacles, offset: FT):
        traits = Arr2.Arr_face_overlay_traits(lambda x, y: x + y)

        arrangements = []
        # build an arrangement for each expanded polygon
        for polygon in obstacles:
            pol = POL2.Polygon_2(polygon)
            ms = MN2.approximated_offset_2(pol, offset, 0.001)
            arr = Arr2.Arrangement_2()
            # Arrangement for the sum
            Arr2.insert(arr, [curve for curve in ms.outer_boundary().curves()])
            for hole in ms.holes():
                Arr2.insert(arr, [curve for curve in hole.curves()])
            ubf = arr.unbounded_face()
            ubf.set_data(0)
            invalid_face = next(next(ubf.inner_ccbs())).twin().face()
            invalid_face.set_data(1)
            for ccb in invalid_face.inner_ccbs():
                valid_face = next(ccb).twin().face()
                valid_face.set_data(0)
            arrangements.append(arr)

        # overlay the arrangements
        initial = Arr2.Arrangement_2()
        ubf = initial.unbounded_face()
        ubf.set_data(0)
        arrangements.insert(0, initial)
        res = None
        for i in range(len(arrangements) - 1):
            res = Arr2.Arrangement_2()
            Arr2.overlay(arrangements[i], arrangements[i + 1], res, traits)
            arrangements[i + 1] = res

        self.cspace = res
        self.pl = Arr2.Arr_trapezoid_ric_point_location(self.cspace)

    def is_edge_valid(self, curve: KER.Segment_2):
        res = []
        if curve.is_degenerate():
            return True
        Arr2.zone(self.cspace, Arr2.X_monotone_curve_2(curve.source(), curve.target()), res, self.pl)
        for obj in res:
            f = Arr2.Face()
            if obj.get_face(f):
                if f.data() > 0:
                    return False
        return True

    def is_point_valid(self, p):
        p = Arr2.Point_2(p[0], p[1])
        obj = self.pl.locate(p)
        f = Arr2.Face()
        if obj.get_face(f):
            if f.data() > 0:
                return False
        return True


# def check_intersection_against_robots(edges, opponent_robots, radius: FT):
#     ms_squared_radius = FT(4) * radius * radius
#     # Check intersection against opponent robots
#     arr = Arrangement_2()
#     for robot in opponent_robots:
#         c = Circle_2(robot[0], ms_squared_radius, CLOCKWISE)
#         insert(arr, Curve_2(c))
#     edge: Segment_2
#     for edge in edges:
#         if not edge.is_degenerate() and do_intersect(arr, X_monotone_curve_2(edge.source(), edge.target())):
#             return True
#     # Check intersection between two robots while following the path
#     for i in range(len(edges)):
#         for j in range(i + 1, len(edges)):
#             arr = Arrangement_2()
#             c = Circle_2(edges[j].source(), ms_squared_radius, CLOCKWISE)
#             insert(arr, Curve_2(c))
#             v1 = Vector_2(edges[i])
#             v2 = Vector_2(edges[j])
#             v = v1 - v2
#             cv = X_monotone_curve_2(edges[i].source(), edges[i].source() + v)
#             if cv.source() != cv.target():
#                 if do_intersect(arr, cv):
#                     return True
#     return False
