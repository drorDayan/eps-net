# from math import sqrt
# from CGALPY.Arr2 import *
# from CGALPY.Ker import *
# from CGALPY.Pol2 import *
# from CGALPY.BSO2 import *
# from CGALPY.CH2 import *
# from CGALPY.SS import *
# from CGALPY.BV import *
# from CGALPY.MN2 import *
# from CGALPY.PP2 import *
# from CGALPY.Tri2 import *
#
# FREESPACE = 'freespace'
#
#
# # noinspection PyArgumentList
# def polygon_with_holes_to_arrangement(poly):
#     assert isinstance(poly, Polygon_with_holes_2)
#     arr = Arrangement_2()
#     insert(arr, [Curve_2(edge) for edge in poly.outer_boundary().edges()])
#
#     # set the freespace flag for the only current two faces
#     for f in arr.faces():
#         assert isinstance(f, Face)
#         f.set_data({FREESPACE: f.is_unbounded()})
#
#     for hole in poly.holes():
#         insert(arr, [Curve_2(edge) for edge in hole.edges()])
#
#     for f in arr.faces():
#         assert isinstance(f, Face)
#         if f.data() is None:
#             f.set_data({FREESPACE: True})
#     return arr
#
#
# def get_origin_robot_coord(width):
#     robot_width = width / FT(2)
#     v1 = Point_2(robot_width, robot_width)
#     v2 = Point_2(robot_width * FT(-1), robot_width)
#     v3 = Point_2(robot_width * FT(-1), robot_width * FT(-1))
#     v4 = Point_2(robot_width, robot_width * FT(-1))
#     return [v1, v2, v3, v4]
#
#
# def merge_faces_by_freespace_flag(x, y):
#     return {FREESPACE: x[FREESPACE] and y[FREESPACE]}
#
#
# # noinspection PyArgumentList
# def overlay_multiple_arrangements(arrs, face_merge_func):
#     final_arr = arrs[0]
#     for arr in arrs[1:]:
#         temp_res = Arrangement_2()
#
#         overlay(final_arr, arr, temp_res, Arr_face_overlay_traits(face_merge_func))
#         final_arr = temp_res
#     return final_arr
#
#
# # noinspection PyArgumentList
# def is_in_free_face(point_locator, point):
#     face = Face()
#     # locate can return a vertex or an edge or a face
#     located_obj = point_locator.locate(point)
#     # if located_obj.is_vertex():
#     #     return False
#     # if located_obj.is_halfedge():
#     #     return False
#     if located_obj.is_face():
#         located_obj.get_face(face)
#         return face.data()[FREESPACE]
#     return False
#
#
# # noinspection PyArgumentList
# def vertical_decompose(arr):
#     assert isinstance(arr, Arrangement_2)
#     d = []
#     verticals = Arrangement_2()
#     decompose(arr, d)
#     for pair in d:
#         # pair is a tuple
#         # pair[0] is an arrangement vertex
#         # pair[1] is a pair holding the objects (vertex, halfedge, or face) above and below the vertex,
#         # that is, the objects hit by the vertical walls emanating from the vertex
#         v0 = pair[0]
#         for obj in pair[1]:
#             if obj.is_vertex():
#                 v1 = Vertex()
#                 obj.get_vertex(v1)
#                 insert(verticals, Curve_2(Segment_2(v0.point(), v1.point())))
#             elif obj.is_halfedge():
#                 he = Halfedge()
#                 obj.get_halfedge(he)
#                 v1 = Point_2(v0.point().x(), he.curve().line().y_at_x(v0.point().x()))
#                 insert(verticals, Curve_2(Segment_2(v0.point(), v1)))
#             else:  # obj is a face
#                 # can only happen for the vertices of the bbox, so IGNORE
#                 pass
#
#     res = Arrangement_2()
#     for f in verticals.faces():
#         f.set_data({FREESPACE: True})
#     overlay(arr, verticals, res, Arr_face_overlay_traits(merge_faces_by_freespace_flag))
#
#     return res
#
#
# # obs collision detection code:
# # noinspection PyArgumentList
# class SRCollisionDetectorFast:
#     def __init__(self, robot_width, obstacles):
#         # init obs for collision detection
#         one_width_square = Polygon_2(get_origin_robot_coord(robot_width))
#         inflated_obstacles = [Polygon_2([p for p in obs]) for obs in obstacles]
#         c_space_obstacles = [minkowski_sum_by_full_convolution_2(one_width_square, obs) for obs in inflated_obstacles]
#         c_space_arrangements = [polygon_with_holes_to_arrangement(obs) for obs in c_space_obstacles]
#         self.obstacles_arrangement = overlay_multiple_arrangements(c_space_arrangements, merge_faces_by_freespace_flag)
#         self.obstacles_point_locator = Arr_trapezoid_ric_point_location(self.obstacles_arrangement)
#         self.robot_width = robot_width.to_double()
#
#     def is_valid_conf(self, p):
#         if isinstance(p, Point_d):
#             p = Point_2(p[0], p[1])
#         return is_in_free_face(self.obstacles_point_locator, p)
#
#     # checks for collisions return:
#     # True if collision free
#     # False, if not
#     def path_collision_free(self, p1, p2):
#         if isinstance(p1, Point_d):
#             p1 = Point_2(p1[0], p1[1])
#         if isinstance(p2, Point_d):
#             p2 = Point_2(p2[0], p2[1])
#
#         # check for obs collision
#         return not do_intersect(self.obstacles_arrangement, Curve_2(p1, p2))
#
#
