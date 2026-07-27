"""Lane-offset facts omitted by the legacy ETS2LA 1.59 JSON exporter.

The values below are the first component of ``lane_offsets_left/right`` from
the official ETS2 1.59 ``def/world/road_look*.sii`` files.  The old map JSON
retained ``road_offset`` and lane types but dropped these arrays, which moves
some carriageways by as much as one or two complete lanes.  New TruckLib map
format v1 datasets carry the arrays directly and never use this compatibility
table.
"""


LANE_OFFSETS_159 = {}


def _add(tokens, left, right):
    for token in tokens.split():
        LANE_OFFSETS_159[token] = (tuple(left), tuple(right))


_add("balt11 balt38 blke15 blke27 blkw1md blkw1min ger37 ger4 ger6 "
     "ibe16 template26 template34 template48 template55", (-1.0,), (-1.0,))
_add("blke16 blke17 blke35 blke37 blke37b blkw2 blkw2c blkw2no "
     "ger30 ger31 ger32", (-4.75, -4.75), (-4.75, -4.75))
_add("balt9 balt9_exp blke2 blke3 blkwhw11 blkwhw15 blkwhw1no ger22 "
     "template21 template45 un11_2m", (-4.5,), (-4.5,))
_add("balt42 blke9 blkw3 blkwhw32 blkwhw35 ger20 un33_10m un33_2m "
     "un33_4m un33_tdai", (-9.0, -9.0, -9.0), (-9.0, -9.0, -9.0))
_add("blke6 blke6_l blke7 blkwhw22 blkwhw25 ibe6 un22_10m un22_2m "
     "un22_4m un22_tdai", (-4.5, -4.5), (-4.5, -4.5))
_add("blke34 blke34w blkwrai ibe32 template65 unrai", (), (-0.5,))
_add("balt43 blke23 template62 template62a template98 template98b",
     (0.0,), (0.0,))
_add("balt32 blke25 blkw1p1 ger26 ibe24 un101", (2.25,), (2.25,))
_add("blke12 blke13 blkwhw42 blkwhw45 un44_2m",
     (-9.0, -9.0, -9.0, -9.0), (-9.0, -9.0, -9.0, -9.0))
_add("blke33 blke33b blkw1ai blkwaid unrdai", (-5.0,), (-5.0,))
_add("blke21 ger36 tram42 untai", (), (1.75,))
_add("ger5 template32 template53 template61", (), (2.25,))
_add("balt17 blke19 ger33 untdai", (-2.75,), (-2.75,))
_add("balt39 blke28 blkw1mdc ger39", (-2.5,), (-2.5,))
_add("blkw1n un11_nar un11_nar_d un11_nar_in", (-0.5,), (-0.5,))
_add("un11_min un11_min_d un11_min_in", (-1.5,), (-1.5,))
_add("blke18 blke18b", (-8.5, -9.25, -9.25),
     (-8.5, -9.25, -9.25))
_add("ger34 tram41", (-3.75, -4.5, -4.5), (-3.75, -4.5, -4.5))
_add("un11_onelane un11_spec", (-2.1,), (-2.1,))
_add("template33 template54", (-0.99,), (-0.99,))
_add("blkw2p1no ger38", (4.5,), ())
_add("template99", (), (0.0,))
_add("ger27", (-9.0, -9.0, -9.0), (-4.5, -4.5))
_add("blke10", (-9.0, -9.0, -9.0, -9.0), (-9.0, -9.0, -9.0))
_add("blkw2p1p2", (-0.15, -0.45), (-0.15, -0.45, -0.75))
_add("blke39", (0.5,), (0.0, 0.0, 0.25))


del _add
